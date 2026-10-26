"""Shared settings for limit_setter.py, move_to_angle.py (library) and app.py."""
import os

PORT = os.environ.get("ST3215_PORT", "/dev/ttyACM0")
SERVO_IDS = [1, 2, 3, 4]

# How each motor is mounted.
#   "normal"   : standard mounting. Positive angle = CLOCKWISE.
#   "reversed" : mounted the opposite way. Positive angle = ANTICLOCKWISE.
# In both cases positive angle = the servo's position reading INCREASING.
MOUNTING = {1: "normal", 2: "reversed", 3: "reversed", 4: "reversed"}

# --- used when the planar-arm app (app.py) drives the servos -------------------
# Which servo drives each joint (theta1, theta2, theta3).
JOINT_TO_SERVO = (1, 2, 3)

# Servo that drives the gripper (controlled by the slider in the app).
GRIPPER_SERVO = 4

# The kinematics in kinematics.py / app.py is counter-clockwise positive.
# Servos are positive clockwise ("normal") or anticlockwise ("reversed"), so the
# library flips the sign for "normal" servos automatically. If a joint still
# moves the wrong way, force its sign here, e.g. {2: -1}.
APP_ANGLES_CCW_POSITIVE = True
JOINT_SIGN_OVERRIDE = {}

# Motion profile while streaming targets at ~30 Hz. High speed and zero
# acceleration make the servo follow the app's own easing instead of adding another.
STREAM_SPEED = 3000   # steps/s
STREAM_ACCEL = 0      # 0 = no acceleration ramp


def positive_direction(sid):
    return "CLOCKWISE" if MOUNTING.get(sid, "normal") == "normal" else "ANTICLOCKWISE"