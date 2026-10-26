"""ServoArm: talks to the ST3215 servos. Used by hardware_link.py.

WORKFLOW
    arm = ServoArm(servo_ids=...).connect()   # open port, read limits from servos
    arm.engage()                              # hold current pose, torque ON (no jump)
    arm.send_joint_angles_rad((t1, t2, t3))   # call every frame (fast, no readback)
    arm.close()                               # close port (torque stays on)

    Every angle sent is converted app -> servo convention (joint_sign),
    turned into steps, clamped to the servo's stored limits, and broadcast.

Run directly for a simple command line:  python3 move_to_angle.py
"""
import math
import re
import threading
import time

from python_st3215 import ST3215
from servo_config import (
    PORT, SERVO_IDS, JOINT_TO_SERVO, GRIPPER_SERVO, APP_ANGLES_CCW_POSITIVE,
    JOINT_SIGN_OVERRIDE, STREAM_SPEED, STREAM_ACCEL, positive_direction,
)

CENTER = 2048                    # corrected reading that means 0 degrees
STEPS_PER_DEG = 4096 / 360.0
TOLERANCE = 3                    # steps (~0.26 deg)


# ------------------------------------------------------------ conversions
def wrap(v):
    return ((v + 2048) % 4096) - 2048


def steps_to_deg(steps):
    return wrap(steps - CENTER) / STEPS_PER_DEG


def deg_to_steps(deg):
    return int(round(CENTER + deg * STEPS_PER_DEG))


def joint_sign(sid):
    """+1/-1: converts between the app's angle direction and this servo's."""
    if sid in JOINT_SIGN_OVERRIDE:
        return JOINT_SIGN_OVERRIDE[sid]
    servo_is_ccw = positive_direction(sid) == "ANTICLOCKWISE"
    return 1 if servo_is_ccw == APP_ANGLES_CCW_POSITIVE else -1


def retry(fn, tries=5):
    """Call fn until it returns a value (bus errors are common). None if all fail."""
    for _ in range(tries):
        try:
            v = fn()
            if v is not None:
                return v
        except Exception:
            pass
        time.sleep(0.05)
    return None


class ServoArm:
    def __init__(self, port=PORT, servo_ids=None, joint_to_servo=JOINT_TO_SERVO):
        self.port = port
        self.ids = list(servo_ids if servo_ids is not None else SERVO_IDS)
        self.joint_to_servo = tuple(joint_to_servo)
        self.controller = None
        self.servos = {}
        self.broadcast = None
        self.limits = {}            # sid -> {lo, hi, min_deg, max_deg, offset}
        self.stream_errors = 0      # consecutive failed sends (hardware_link checks this)
        self._bus_lock = threading.RLock()

    # ------------------------------------------------------------------
    # 1. Connection
    # ------------------------------------------------------------------
    def connect(self):
        self.controller = ST3215(self.port)
        try:
            self.servos = {sid: self.controller.wrap_servo(sid) for sid in self.ids}
            self.broadcast = self.controller.broadcast
            self._read_limits()
        except Exception:
            self.close()
            raise
        return self

    def close(self):
        """Close the port. Torque is left on so the arm keeps holding."""
        if self.controller is not None:
            self.controller.close()
            self.controller = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
        return False

    def _read_limits(self):
        """Read limits + offset stored in each servo's EEPROM (once)."""
        for sid in self.ids:
            eeprom = self.servos[sid].eeprom
            lo = retry(eeprom.read_min_angle_limit)
            hi = retry(eeprom.read_max_angle_limit)
            if lo is None or hi is None:
                raise RuntimeError(f"Could not read limits of servo {sid}")
            a, b = steps_to_deg(lo), steps_to_deg(hi)
            self.limits[sid] = dict(lo=lo, hi=hi, min_deg=min(a, b), max_deg=max(a, b),
                                    offset=retry(eeprom.read_position_correction))

    # ------------------------------------------------------------------
    # 2. Reading
    # ------------------------------------------------------------------
    def read_steps(self, sid):
        with self._bus_lock:
            pos = retry(self.servos[sid].sram.read_current_location)
        return None if pos is None else pos % 4096

    def read_joint_angles_rad(self):
        """(t1, t2, t3) in the app's convention, or None if any read failed."""
        out = []
        for sid in self.joint_to_servo:
            s = self.read_steps(sid)
            if s is None:
                return None
            out.append(math.radians(joint_sign(sid) * steps_to_deg(s)))
        return tuple(out)

    def joint_limits_rad(self):
        """[(min, max)] per joint in the app's convention (flipped if signs differ)."""
        out = []
        for sid in self.joint_to_servo:
            lim, sign = self.limits[sid], joint_sign(sid)
            a, b = sign * lim["min_deg"], sign * lim["max_deg"]
            out.append((math.radians(min(a, b)), math.radians(max(a, b))))
        return out

    # ------------------------------------------------------------------
    # 3. Motion
    # ------------------------------------------------------------------
    def engage(self, ids=None, speed=STREAM_SPEED, accel=STREAM_ACCEL):
        """Aim each servo at where it is now (no jump), set speed, torque ON."""
        ids = ids or self.ids
        with self._bus_lock:
            targets = {}
            for sid in ids:
                s = self.read_steps(sid)
                if s is None:
                    raise RuntimeError(f"Could not read position of servo {sid}")
                lim = self.limits[sid]
                targets[sid] = max(lim["lo"], min(lim["hi"], s))
            sram = self.broadcast.sram
            sram.sync_write_target_location(targets)
            sram.sync_write_acceleration({sid: accel for sid in ids})
            sram.sync_write_running_speed({sid: speed for sid in ids})
            sram.sync_write_torque_switch({sid: 1 for sid in ids})

    def send_angles(self, angles_deg):
        """FAST: one broadcast packet, no readback. {servo_id: degrees} in the servo's
        convention. Clamped to limits. Returns {servo_id: (applied_deg, was_clamped)}."""
        targets, applied = {}, {}
        for sid, deg in angles_deg.items():
            if deg is None:
                continue
            lim = self.limits[sid]
            want = deg_to_steps(deg)
            steps = max(lim["lo"], min(lim["hi"], want))
            targets[sid] = steps
            applied[sid] = (steps_to_deg(steps), steps != want)
        if targets:
            try:
                with self._bus_lock:
                    self.broadcast.sram.sync_write_target_location(targets)
                self.stream_errors = 0
            except Exception:
                self.stream_errors += 1
                raise
        return applied

    def send_joint_angles_rad(self, thetas):
        """thetas = (t1, t2, t3) radians, app convention."""
        return self.send_angles({sid: joint_sign(sid) * math.degrees(t)
                                 for sid, t in zip(self.joint_to_servo, thetas)})

    # ------------------------------------------------------------------
    # 4. Gripper
    # ------------------------------------------------------------------
    def gripper_range_deg(self, sid=GRIPPER_SERVO):
        lim = self.limits[sid]
        return lim["min_deg"], lim["max_deg"]

    def read_gripper_deg(self, sid=GRIPPER_SERVO):
        s = self.read_steps(sid)
        return None if s is None else steps_to_deg(s)

    def send_gripper_deg(self, deg, sid=GRIPPER_SERVO):
        return self.send_angles({sid: deg})[sid]

    # ------------------------------------------------------------------
    # 5. Blocking move (command line only)
    # ------------------------------------------------------------------
    def move_and_wait(self, angles_deg, speed=1000, accel=50, timeout=8.0, log=print):
        """Check everything first (nothing moves if any angle is invalid), move,
        then wait until every servo arrives. Returns True on success."""
        plan = {}                                   # sid -> (current_deg, wanted_deg, target_steps)
        for sid, ang in angles_deg.items():
            if ang is None:
                continue
            s = self.read_steps(sid)
            if s is None:
                log(f"  Servo {sid}: could not read status. Nothing was moved.")
                return False
            lim = self.limits[sid]
            if not (lim["min_deg"] - 1e-6 <= ang <= lim["max_deg"] + 1e-6):
                log(f"  Servo {sid}: {ang:.2f} deg is outside "
                    f"[{lim['min_deg']:.2f}, {lim['max_deg']:.2f}]. Nothing was moved.")
                return False
            target = max(lim["lo"], min(lim["hi"], deg_to_steps(ang)))
            if abs(wrap(s - target)) > TOLERANCE:
                plan[sid] = (steps_to_deg(s), ang, target)

        if not plan:
            log("  No move needed.")
            return True

        self.engage(list(plan), speed=speed, accel=accel)
        for sid, (cur, ang, target) in plan.items():
            log(f"  Servo {sid}: {cur:.2f} -> {ang:.2f} deg")
        with self._bus_lock:
            self.broadcast.sram.sync_write_target_location({s: p[2] for s, p in plan.items()})

        pending, t0 = set(plan), time.time()
        while pending and time.time() - t0 < timeout:
            time.sleep(0.1)
            for sid in list(pending):
                s = self.read_steps(sid)
                if s is not None and abs(wrap(s - plan[sid][2])) <= TOLERANCE:
                    pending.discard(sid)
        if pending:
            log(f"  WARNING: timed out waiting for servo(s) {sorted(pending)}.")
            return False
        log("  All servos reached their targets.")
        return True


# =============================== command line ================================
def show_all(arm):
    print("-" * 70)
    print(f"{'Servo':<6}{'Positive':<15}{'Now':>9}{'Min':>9}{'Max':>9}{'Offset':>9}")
    for sid in arm.ids:
        s = arm.read_steps(sid)
        lim = arm.limits[sid]
        now = "n/a" if s is None else f"{steps_to_deg(s):.2f}"
        print(f"{sid:<6}{positive_direction(sid):<15}{now:>9}"
              f"{lim['min_deg']:>9.2f}{lim['max_deg']:>9.2f}{lim['offset']:>9}")
    print("-" * 70)


def parse_angles(text, n):
    tokens = [t for t in re.split(r"[,\s]+", text.strip()) if t]
    if len(tokens) != n:
        raise ValueError(f"need exactly {n} values (use '-' to skip), got {len(tokens)}")
    return [None if t in ("-", "x", "skip") else float(t) for t in tokens]


def main():
    with ServoArm() as arm:
        show_all(arm)
        order = " ".join(f"S{sid}" for sid in arm.ids)
        print(f"Enter {len(arm.ids)} angles (deg) in order [{order}]. '-' skips a servo, 'q' quits.")
        while True:
            cmd = input("\nAngles: ").strip().lower()
            if cmd == "q":
                break
            try:
                vals = parse_angles(cmd, len(arm.ids))
                arm.move_and_wait(dict(zip(arm.ids, vals)))
            except Exception as e:
                print(f"  ERROR: {e}")
            show_all(arm)
        print("Done. Torque left on so the arm holds its position.")


if __name__ == "__main__":
    main()