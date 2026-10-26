"""UI Glue: Handles widget signals and syncs with HardwareLink."""
import math

from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QTimer

from config_manager import load_config
from app.arm_widget import ArmPlotWidget
from app.controls_widget import ControlsWidget
from hardware_link import HardwareLink

UI_INTERVAL_MS = 33  # ~30 FPS


class ArmControllerWindow(QMainWindow):
    def __init__(self, config_file="config.yaml"):
        super().__init__()
        self.setWindowTitle("Robotic Arm PyQt Controller")
        self.resize(1280, 860)

        self.config_data = load_config(config_file)
        self.hw = HardwareLink(self.config_data)

        self.target_x = self.config_data['initial_target']['x']
        self.target_y = self.config_data['initial_target']['y']
        self.target_phi = math.radians(45)
        self.target_elbow_up = True
        self.target_valid = True
        self.auto_phi = True
        self.track_path = True
        self.waypoints = []
        self.loop_waypoints = False

        # Dirty-check caches for sync_ui
        self._last_status = None
        self._last_angles = None
        self._last_trace_len = -1
        self._last_hist_len = -1
        self._last_track = None

        self.hw.set_offline()
        self._build_ui()

        self._apply_feasible_config()
        self.update_target_display()
        self._move_to_target()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.sync_ui)
        self.timer.start(UI_INTERVAL_MS)

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(8)

        self.lbl_status = QLabel("Ready")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setMinimumHeight(30)
        self.lbl_status.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: blue;"
            "border: 1px solid #c0c0c0; border-radius: 4px; padding: 4px;")
        layout.addWidget(self.lbl_status)

        self.plot = ArmPlotWidget(self.hw.arm)
        self.plot.update_limit_display(self.hw.arm.joint_limits)
        layout.addWidget(self.plot, stretch=1)
        self.plot.canvas.mpl_connect('button_press_event', self.on_click)

        self.controls = ControlsWidget(self.hw.arm, {
            'bezier_weight': self.hw.bezier_weight,
            'duration_s': self.hw.duration_s,
            'dwell_s': 0.0,
            'xlim': self.plot.custom_xlim,
            'ylim': self.plot.custom_ylim,
        })
        layout.addWidget(self.controls)

        c = self.controls
        c.paramsChanged.connect(self.update_params)
        c.phiChanged.connect(self.on_phi_dial)
        c.elbowChanged.connect(self.on_elbow_toggle)
        c.gripperChanged.connect(self.hw.send_gripper)
        c.modeToggled.connect(self.on_mode_toggle)
        c.addWaypointClicked.connect(self.add_waypoint)
        c.undoWaypointClicked.connect(self.undo_waypoint)
        c.clearWaypointsClicked.connect(self.clear_waypoints)
        c.moveClicked.connect(self._move_to_target)
        c.runWaypointsClicked.connect(self._run_waypoints)
        c.stopClicked.connect(self.hw.stop)
        c.loopToggled.connect(self.toggle_loop)
        c.autoPhiToggled.connect(self.on_auto_phi_toggle)
        c.trackPathToggled.connect(self.on_track_toggle)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _move_to_target(self):
        self.hw.move_to_target(self.target_x, self.target_y,
                               self.target_phi, self.target_elbow_up)

    def _run_waypoints(self):
        self.hw.run_waypoints(self.waypoints, self.loop_waypoints)

    def _apply_feasible_config(self, x=None, y=None):
        """Find feasible phi/elbow for (x, y) or current target; sync widgets."""
        x = self.target_x if x is None else x
        y = self.target_y if y is None else y
        phi, elbow = self.hw.find_feasible_config(
            x, y, self.target_phi, self.target_elbow_up, self.auto_phi)
        if phi is not None:
            self.target_phi, self.target_elbow_up = phi, elbow
            self.controls.set_phi_silently(math.degrees(phi))
            self.controls.set_elbow_silently(elbow)
        return phi is not None

    # ------------------------------------------------------------------
    def sync_ui(self):
        """Pulls state from HardwareLink; redraws only when something changed."""
        st = self.hw.get_state()
        self.hw.check_health()
        dirty = False

        if st['status'] != self._last_status:
            self._last_status = st['status']
            text, color = st['status']
            self.lbl_status.setText(text)
            self.lbl_status.setStyleSheet(
                f"font-size: 14px; font-weight: bold; color: {color};"
                "border: 1px solid #c0c0c0; border-radius: 4px; padding: 4px;")

        angles = tuple(st['angles'])
        if angles != self._last_angles:
            self._last_angles = angles
            self.plot.draw_arm(*angles)
            dirty = True

        trace = st['trace'] if self.track_path else []
        if len(trace) != self._last_trace_len or self.track_path != self._last_track:
            self._last_trace_len = len(trace)
            self._last_track = self.track_path
            self.plot.set_trace(trace)
            dirty = True

        hist_len = len(st['hist_t'])
        if hist_len != self._last_hist_len:
            self._last_hist_len = hist_len
            self.plot.refresh_graphs(st['hist_t'], st['hist_a'])
            dirty = True

        if not st['is_online'] and self.controls.btn_mode.isChecked():
            self.controls.set_mode_button(False)

        if dirty:
            self.plot.canvas.draw_idle()

    # ------------------------------------------------------------------
    def update_params(self):
        c = self.controls
        try:
            arm = self.hw.arm
            arm.L1, arm.L2, arm.L3 = (float(t.text()) for t in (c.tb_l1, c.tb_l2, c.tb_l3))
            arm.x_min, arm.y_min = float(c.tb_xmin.text()), float(c.tb_ymin.text())
            self.hw.bezier_weight = float(c.tb_bez.text())
            self.hw.duration_s = max(0.1, float(c.tb_dur.text()))
            xlim, ylim = float(c.tb_xlim.text()), float(c.tb_ylim.text())
        except ValueError:
            self.lbl_status.setText("Invalid input! Numbers only.")
            return

        self.plot.set_plot_limits(xlim, ylim)
        self.plot.set_target_angles(None, None, None, valid=False)
        self.plot.redraw_waypoints(self.waypoints, draw=False)  # setup_plot cleared artists
        self._last_trace_len = -1

        if self._apply_feasible_config():
            self.update_target_display()
        else:
            self.plot.target_point.set_color('gray')
            self.lbl_status.setText("Target violates new constraints!")

    def on_phi_dial(self, val):
        self.target_phi = math.radians(val)
        self.plot.clear_ghost_arm()
        self.update_target_display()

    def on_elbow_toggle(self, checked):
        self.target_elbow_up = checked
        self._apply_feasible_config()
        self.plot.clear_ghost_arm()
        self.update_target_display()

    def on_mode_toggle(self, checked):
        success = self.hw.set_online() if checked else self.hw.set_offline()
        if not success:
            self.controls.revert_mode_button()
        else:
            self.controls.set_mode_button(checked)
            self.plot.update_limit_display(self.hw.arm.joint_limits)

    def on_auto_phi_toggle(self, checked):
        self.auto_phi = checked

    def on_track_toggle(self, checked):
        self.track_path = checked
        if not checked:
            self.plot.set_trace([])

    def on_click(self, event):
        if event.inaxes != self.plot.ax or self.hw.state['is_moving']:
            return
        x, y = event.xdata, event.ydata

        if self._apply_feasible_config(x, y):
            self.target_x, self.target_y = x, y
            self.target_valid = True
            self.update_target_display()
            solved = self.hw.arm.inverse_kinematics(
                x, y, self.target_phi, self.target_elbow_up)
            self.plot.draw_ghost_arm(*solved)
            self.plot.show_ghost_target(x, y)
        else:
            self.plot.set_target_marker(x, y, valid=False)
            self.plot.ghost_arm_line.set_data([], [])
            self.plot.set_target_angles(None, None, None, valid=False)
            self.target_valid = False
            self.lbl_status.setText("Unreachable target.")

        self.plot.canvas.draw_idle()

    def update_target_display(self):
        self.target_valid = True
        self.plot.set_target_marker(self.target_x, self.target_y, valid=True)
        try:
            t1, t2, t3 = self.hw.arm.inverse_kinematics(
                self.target_x, self.target_y, self.target_phi, self.target_elbow_up)
            self.plot.set_target_angles(t1, t2, t3, valid=True)
        except ValueError:
            self.plot.set_target_angles(None, None, None, valid=False)

        elbow_str = "Up" if self.target_elbow_up else "Down"
        self.lbl_status.setText(
            f"Target: ({self.target_x:.1f}, {self.target_y:.1f}) | "
            f"Phi: {math.degrees(self.target_phi):.1f}° | Elbow: {elbow_str}")
        self.plot.canvas.draw_idle()

    # ------------------------------------------------------------------
    def add_waypoint(self):
        if not self.target_valid:
            return
        self.waypoints.append({'x': self.target_x, 'y': self.target_y,
                               'phi': self.target_phi,
                               'elbow_up': self.target_elbow_up})
        self.plot.redraw_waypoints(self.waypoints)
        self.lbl_status.setText(f"Waypoint {len(self.waypoints)} added.")

    def undo_waypoint(self):
        if self.waypoints:
            self.waypoints.pop()
            self.plot.redraw_waypoints(self.waypoints)

    def clear_waypoints(self):
        self.waypoints.clear()
        self.plot.set_trace([])
        self.plot.redraw_waypoints(self.waypoints)

    def toggle_loop(self, checked):
        self.loop_waypoints = checked

    def closeEvent(self, event):
        self.timer.stop()
        self.hw.close()
        event.accept()
