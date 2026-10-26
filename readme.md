# Robotic Arm Controller

A desktop app to control a 3-joint robot arm. Click on the plot, and the arm moves there.
It works in **Offline** mode (simulation only) or **Online** mode (moves the real servos).

## How it works

```
 You click / press buttons
          │
   controls_widget.py   buttons, fields, dial
   arm_widget.py        plot of the arm + graphs
          │
     controller.py      connects the screen to the logic
          │
    hardware_link.py    calculates the path, runs the motion
          │
    move_to_angle.py    sends angles to the servos
```

## Files

| File | What it does |
|------|--------------|
| `app_qt.py` | Opens the window. |
| `controller.py` | Handles clicks and updates the screen 30 times a second. |
| `controls_widget.py` | The bottom panel: buttons and input fields. |
| `arm_widget.py` | The arm drawing and the joint graphs. |
| `hardware_link.py` | The brain: kinematics, smooth motion, offline/online mode. |
| `move_to_angle.py` | Talks to the ST3215 servos. |
| `limit_setter.py` | One-time tool to set each servo's zero and limits. |

## Quick start

1. **Calibrate the servos once:** `python limit_setter.py`
   Follow the prompts: move the joint to **min**, then **max**, then **zero**.
2. **Run the app:** `python main.py`
3. **Use it:**
   - Click on the plot to choose a target (a gray ghost arm shows the preview).
   - Press **Move Arm** to go there.
   - Press **Add WP** to save a waypoint, then **Run WPs** to follow them all.
   - Press **Stop** at any time.
   - Switch **Mode: OFFLINE** to **ONLINE** to control the real arm.

## Controls

- **Phi / dial:** angle of the gripper.
- **Elbow UP/DOWN:** which way the elbow bends.
- **Auto Phi:** lets the app change Phi if the target can't be reached.
- **Loop:** repeats the waypoints.
- **Track:** shows the path the tip has drawn.
- **Bezier W / Duration:** how smooth and how long each move takes.

## Tips

- Run **offline first** to test paths safely.
- If the connection drops, the app goes back to offline and shows a red message.
- Needs: `PyQt5`, `matplotlib`, `python_st3215`, and your `config.yaml`, `kinematics.py`, `servo_config.py`.