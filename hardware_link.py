"""HardwareLink: the single bridge between the UI and the arm (real or simulated).

WORKFLOW
    UI click ──> move_to_target() / run_waypoints()
                    │  1. solve inverse kinematics (angles for each pose)
                    │  2. check the path respects the workspace limits
                    ▼
                 background thread  (_run_path, 100 Hz)
                    │  3. ease between poses with a Bezier curve
                    │  4. store angles / trace / history in self.state
                    │  5. send angles to servos (only if online)
                    ▼
    UI timer ──> get_state()   (reads a snapshot, never blocks the motion)

MODES
    offline = simulation only        online = also drives the real servos
"""
import math
import threading
import time
from bisect import bisect_left

from kinematics import Planar3DOFArm

try:
    from move_to_angle import ServoArm
    from servo_config import JOINT_TO_SERVO, GRIPPER_SERVO
except ImportError:
    ServoArm = None
    JOINT_TO_SERVO = ()
    GRIPPER_SERVO = None

LOOP_DT = 0.01          # 100 Hz control loop
HISTORY_S = 25.0        # seconds of joint history kept for the graphs
SNAP_DIST = 5.0         # moves shorter than this just jump to the target


def bezier_ease(t, w):
    """Smooth 0..1 -> 0..1 easing; w controls how sharp the start/stop is."""
    return max(0.0, min(1.0, t * t * (w - (w - 2) * t)))


class HardwareLink:
    # ------------------------------------------------------------------
    # 1. Setup
    # ------------------------------------------------------------------
    def __init__(self, config):
        p, c, a = config['arm_parameters'], config['constraints'], config['animation']
        self.arm = Planar3DOFArm(l1=p['L1'], l2=p['L2'], l3=p['L3'],
                                 x_min=c['min_x'], y_min=c['min_y'])
        self.bezier_weight = a['bezier_weight']
        self.duration_s = a['duration_s']
        self.hw_enabled = config.get('hardware', {}).get('enabled', True)

        self.state = {
            'angles': (0.0, 0.0, 0.0),
            'target_angles': (0.0, 0.0, 0.0),
            'is_moving': False,
            'is_online': False,
            'status': ("Ready", "blue"),
            'trace': [],
            'gripper': 0.0,
            'gripper_range': (-90.0, 90.0),
            'hist_t': [],
            'hist_a': [[], [], []],
        }
        self.abort = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.servo_arm = None
        self._last_gripper_send = 0.0
        self._t0 = time.perf_counter()

    # ------------------------------------------------------------------
    # 2. State access (thread-safe)
    # ------------------------------------------------------------------
    def get_state(self):
        """Snapshot for the UI (lists are copied so the UI can't race the thread)."""
        with self.lock:
            s = dict(self.state)
            s['trace'] = list(s['trace'])
            s['hist_t'] = list(s['hist_t'])
            s['hist_a'] = [list(h) for h in s['hist_a']]
            return s

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def _status(self, text, color="black"):
        self._set(status=(text, color))

    def _record(self, angles):
        """Append to graph history (call with lock held) and trim old data."""
        now = time.perf_counter() - self._t0
        hist_t, hist_a = self.state['hist_t'], self.state['hist_a']
        hist_t.append(now)
        for h, a in zip(hist_a, angles):
            h.append(math.degrees(a))
        cut = bisect_left(hist_t, now - HISTORY_S)
        if cut > 100:  # trim in batches, not one item at a time
            del hist_t[:cut]
            for h in hist_a:
                del h[:cut]

    # ------------------------------------------------------------------
    # 3. Mode switching
    # ------------------------------------------------------------------
    def set_offline(self):
        self._close_hardware()
        lim = math.radians(120)
        self.arm.joint_limits = [(-lim, lim)] * 3
        self._set(is_online=False)
        self._status("Offline mode (Local simulation)", "blue")
        return True

    def set_online(self):
        if not self.hw_enabled or ServoArm is None:
            self._status("Hardware disabled or missing module.", "red")
            return False
        try:
            arm = ServoArm(servo_ids=list(JOINT_TO_SERVO) + [GRIPPER_SERVO]).connect()
            pose = arm.read_joint_angles_rad()
            if pose is None:
                raise RuntimeError("could not read servo positions")

            self.arm.joint_limits = arm.joint_limits_rad()
            lo, hi = arm.gripper_range_deg()
            g = arm.read_gripper_deg()
            with self.lock:
                self.state.update(is_online=True, gripper_range=(lo, hi),
                                  angles=pose, target_angles=pose)
                if g is not None:
                    self.state['gripper'] = max(lo, min(hi, g))
            arm.engage()
            self.servo_arm = arm
            self._status("Online mode (Hardware connected).", "green")
            return True
        except Exception as e:
            self._status(f"Could not connect servos: {e}", "red")
            self.set_offline()
            return False

    def _close_hardware(self):
        if self.servo_arm:
            try:
                self.servo_arm.close()
            except Exception:
                pass
            self.servo_arm = None

    def check_health(self):
        if self.servo_arm and getattr(self.servo_arm, 'stream_errors', 0) >= 5:
            self._close_hardware()
            self._set(is_online=False)
            self._status("Hardware connection lost.", "red")

    def send_gripper(self, val):
        self._set(gripper=float(val))
        now = time.perf_counter()
        if self.servo_arm is None or now - self._last_gripper_send < 0.03:
            return
        self._last_gripper_send = now
        try:
            self.servo_arm.send_gripper_deg(float(val))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 4. Kinematics helpers
    # ------------------------------------------------------------------
    def _joint_points(self, t1, t2, t3):
        """XY of elbow, wrist and tip for given joint angles."""
        a = self.arm
        x1, y1 = a.L1 * math.cos(t1), a.L1 * math.sin(t1)
        x2, y2 = x1 + a.L2 * math.cos(t1 + t2), y1 + a.L2 * math.sin(t1 + t2)
        x3, y3 = x2 + a.L3 * math.cos(t1 + t2 + t3), y2 + a.L3 * math.sin(t1 + t2 + t3)
        return (x1, y1), (x2, y2), (x3, y3)

    def _tip(self, angles):
        return self._joint_points(*angles)[2]

    def find_feasible_config(self, x, y, phi, elbow_up, auto_phi):
        """Return (phi, elbow_up) that can reach (x, y), or (None, None).
        Tries the requested phi first, then nearby values (if auto_phi)."""
        if x < self.arm.x_min or y < self.arm.y_min:
            return None, None
        for off in range(0, 180, 2):
            if off and not auto_phi:
                break
            for sign in (1, -1):
                if off == 0 and sign == -1:
                    continue
                test_phi = self.arm.normalize_angle(phi + math.radians(off * sign))
                for elbow in (elbow_up, not elbow_up):
                    try:
                        self.arm.inverse_kinematics(x, y, test_phi, elbow)
                        return test_phi, elbow
                    except ValueError:
                        pass
        return None, None

    def _path_error(self, poses):
        """Return an error message if any pose along the path leaves the workspace."""
        a = self.arm
        for i, (p, q) in enumerate(zip(poses, poses[1:]), 1):
            for k in range(41):
                f = k / 40
                pts = self._joint_points(*(u + (v - u) * f for u, v in zip(p, q)))
                if (min(x for x, _ in pts) < a.x_min - 1e-6 or
                        min(y for _, y in pts) < a.y_min - 1e-6):
                    return f"Segment {i} violates constraints."
        return None

    # ------------------------------------------------------------------
    # 5. Motion (one thread, one loop for both single moves and waypoints)
    # ------------------------------------------------------------------
    def _run_path(self, poses, loop, label):
        try:
            self._set(trace=[])
            while not self.abort.is_set():
                for i, (start, end) in enumerate(zip(poses, poses[1:]), 1):
                    if len(poses) > 2:
                        self._status(f"Waypoint {i}/{len(poses) - 1}", "orange")
                    if not self._run_segment(start, end):
                        return
                if not loop:
                    break
        finally:
            stopped = self.abort.is_set()
            self._set(is_moving=False,
                      status=("Stopped.", "orange") if stopped else ("Path Complete", "green"))

    def _run_segment(self, start, end):
        """Glide from start to end. Returns False if aborted."""
        t0 = time.perf_counter()
        while not self.abort.is_set():
            ratio = min(1.0, (time.perf_counter() - t0) / self.duration_s)
            ease = bezier_ease(ratio, self.bezier_weight)
            angles = tuple(a + (b - a) * ease for a, b in zip(start, end))

            with self.lock:
                self.state['angles'] = angles
                self.state['trace'].append(self._tip(angles))
                self._record(angles)

            if self.servo_arm:
                try:
                    self.servo_arm.send_joint_angles_rad(angles)
                except Exception:
                    pass
            if ratio >= 1.0:
                return True
            time.sleep(LOOP_DT)
        return False

    def _start(self, poses, loop, status):
        self.abort.clear()
        self._set(is_moving=True, status=(status, "orange"))
        self.thread = threading.Thread(target=self._run_path,
                                       args=(poses, loop, status), daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------
    # 6. Public commands
    # ------------------------------------------------------------------
    def move_to_target(self, x, y, phi, elbow):
        if self.state['is_moving']:
            return
        try:
            target = self.arm.inverse_kinematics(x, y, phi, elbow)
        except ValueError as e:
            self._status(f"IK Error: {e}", "red")
            return

        start = self.state['angles']
        self._set(target_angles=target)

        if math.dist(self._tip(start), self._tip(target)) <= SNAP_DIST:
            self._set(angles=target, status=("Target Reached (Snapped)", "green"))
            if self.servo_arm:
                self.servo_arm.send_joint_angles_rad(target)
            return
        self._start([start, target], False, "Moving (Bezier)...")

    def run_waypoints(self, waypoints, loop):
        if self.state['is_moving'] or not waypoints:
            return
        try:
            solved = [self.arm.inverse_kinematics(w['x'], w['y'], w['phi'], w['elbow_up'])
                      for w in waypoints]
        except ValueError as e:
            self._status(f"Waypoint IK Error: {e}", "red")
            return

        poses = [self.state['angles']] + solved
        if loop and len(solved) > 1:
            poses.append(solved[0])

        err = self._path_error(poses)
        if err:
            self._status(err, "red")
            return
        self._start(poses, loop, "Running waypoints...")

    def stop(self):
        self.abort.set()

    def close(self):
        self.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join()
        self._close_hardware()
