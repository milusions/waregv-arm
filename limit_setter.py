"""Servo calibration tool: sets each servo's zero point and travel limits.

WORKFLOW (repeat per servo)
    1. Pick a servo ID.
    2. Reset its old offset + limits.
    3. Move the joint by hand to: MIN end  ->  MAX end  ->  ZERO position.
       (type the word each time to record the reading)
    4. The script works out which way the travel goes (zero must lie between the ends).
    5. It writes the zero offset, so zero reads CENTER (2048).
    6. It writes min/max limits around CENTER.
"""
import time
from python_st3215 import ST3215
from servo_config import PORT, SERVO_IDS, positive_direction

DEBUG = True
CENTER = 2048        # reading the zero position will be mapped to
DELAY = 0.1          # time for the servo to finish an EEPROM write
STEPS_TO_DEG = 360 / 4096


def dbg(*args):
    if DEBUG:
        print("[DEBUG]", *args)


def wrap(v):
    """Shortest signed distance on the 4096-step circle (-2048..2047)."""
    return ((v + 2048) % 4096) - 2048


# ---------------------------------------------------------------- servo I/O
def write_eeprom(servo, writer, value):
    servo.sram.unlock()
    time.sleep(0.02)
    writer(value)
    time.sleep(DELAY)
    servo.sram.lock()
    time.sleep(DELAY)


def read_position(servo, tries=5):
    """Read position, retrying on bus errors. Returns None if all tries fail."""
    for i in range(tries):
        try:
            pos = servo.sram.read_current_location()
            if pos is not None:
                return pos
        except Exception as e:
            dbg(f"read failed ({i + 1}/{tries}): {e}")
        time.sleep(0.1)
    return None


def dump_state(servo, tag):
    reads = {
        "position": servo.sram.read_current_location,
        "min_limit": servo.eeprom.read_min_angle_limit,
        "max_limit": servo.eeprom.read_max_angle_limit,
        "pos_correction": servo.eeprom.read_position_correction,
    }
    for name, fn in reads.items():
        try:
            dbg(f"{tag}: {name} = {fn()!r}")
        except Exception as e:
            dbg(f"{tag}: {name} read failed ({e})")
        time.sleep(0.02)


def reset_servo(servo):
    write_eeprom(servo, servo.eeprom.write_position_correction, 0)
    write_eeprom(servo, servo.eeprom.write_min_angle_limit, 0)
    write_eeprom(servo, servo.eeprom.write_max_angle_limit, 4095)


# ------------------------------------------------------------------ steps
def capture(servo, label):
    """Ask the user to place the joint, then record its position."""
    while True:
        if input(f"Move to {label.upper()} position, then type '{label}': ").strip().lower() != label:
            print(f"Please type '{label}'.")
            continue
        pos = read_position(servo)
        if pos is None:
            print("Could not read position. Check connection.")
            continue
        print(f"Captured {label} = {pos}")
        return pos


def work_out_travel(raw_min, raw_max, raw_zero):
    """Return (raw_min, raw_max, a, b): a = steps min->zero, b = steps zero->max.
    The reading wraps at 4095, so the arc that contains zero is the real travel."""
    if (raw_max - raw_min) % 4096 < 10:
        raise ValueError("Min and max are (almost) the same position.")
    if (raw_zero - raw_min) % 4096 > (raw_max - raw_min) % 4096:
        print("NOTE: reading went DOWN from first to second end; swapping min/max.")
        raw_min, raw_max = raw_max, raw_min
    a = (raw_zero - raw_min) % 4096
    b = (raw_max - raw_zero) % 4096
    if a > 2048 or b > 2047:
        raise ValueError(f"Travel doesn't fit: {-a * STEPS_TO_DEG:.1f} to "
                         f"{b * STEPS_TO_DEG:.1f} deg from zero (max is +/-180). "
                         "Choose a zero closer to the middle.")
    return raw_min, raw_max, a, b


def write_zero_offset(servo, raw_zero):
    """Shift readings so the current (zero) position reads CENTER."""
    desired = wrap(CENTER - raw_zero)
    offset = max(-2047, min(2047, desired))
    before = read_position(servo)
    write_eeprom(servo, servo.eeprom.write_position_correction, offset)
    after = read_position(servo)

    if before is not None and after is not None:
        shift = wrap(after - before)
        if abs(wrap(shift - desired)) > 5:
            if abs(wrap(shift + desired)) <= 5:      # servo uses the opposite sign
                offset = -offset
                write_eeprom(servo, servo.eeprom.write_position_correction, offset)
            else:
                print(f"WARNING: observed shift {shift} matches neither {desired} nor "
                      f"{-desired}. Did the shaft move?")
    return offset


def calibrate(servo, sid):
    print(f"Servo {sid}: positive direction is {positive_direction(sid)}.")

    # Step 2: reset
    reset_servo(servo)
    print("Offset and limits cleared.")

    # Step 3: capture the three positions
    raw_min = capture(servo, "min")
    raw_max = capture(servo, "max")
    raw_zero = capture(servo, "zero")

    # Step 4: work out the travel
    raw_min, raw_max, a, b = work_out_travel(raw_min, raw_max, raw_zero)
    print(f"Travel: {a + b} steps ({(a + b) * STEPS_TO_DEG:.0f} deg): "
          f"min = -{a * STEPS_TO_DEG:.1f}, max = +{b * STEPS_TO_DEG:.1f} deg from zero.")

    # Step 5 + 6: write offset, then limits
    offset = write_zero_offset(servo, raw_zero)
    write_eeprom(servo, servo.eeprom.write_min_angle_limit, CENTER - a)
    write_eeprom(servo, servo.eeprom.write_max_angle_limit, CENTER + b)
    dump_state(servo, "final")

    print("-" * 50)
    print(f"Servo {sid} done | raw min/max {raw_min}/{raw_max} | zero {raw_zero} | "
          f"offset {offset} | limits {CENTER - a}..{CENTER + b}")
    print("-" * 50)


# ------------------------------------------------------------------- main
def main():
    with ST3215(PORT) as controller:
        servos = {sid: controller.wrap_servo(sid) for sid in SERVO_IDS}
        for servo in servos.values():
            servo.sram.torque_disable()
        print("Torque disabled: you can move the joints by hand.")

        while True:
            print("\n" + "=" * 50)
            s = input(f"Servo ID {SERVO_IDS} (or 'q' to quit): ").strip().lower()
            if s == "q":
                break
            if not s.isdigit() or int(s) not in SERVO_IDS:
                print("Invalid ID.")
                continue
            try:
                calibrate(servos[int(s)], int(s))
            except Exception as e:
                print(f"ERROR on servo {s}: {e}")
                dump_state(servos[int(s)], "after error")
        print("Calibration session finished.")


if __name__ == "__main__":
    main()
