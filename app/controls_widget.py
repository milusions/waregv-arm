"""Bottom panel with all interactive controls, grouped by function."""
from PyQt5.QtWidgets import (QWidget, QGridLayout, QHBoxLayout, QVBoxLayout,
                             QLabel, QLineEdit, QPushButton, QSlider, QDial,
                             QGroupBox, QSizePolicy)
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import Qt, pyqtSignal

# Style constants (built once)
BOLD = "font-weight: bold;"
S_MODE_OFF = f"background-color: lightgray; {BOLD}"
S_MODE_ON = f"background-color: lightgreen; {BOLD}"
S_AUTO_ON = f"background-color: lightblue; {BOLD}"
S_AUTO_OFF = f"background-color: lightyellow; {BOLD}"
S_TRACK_ON = f"background-color: lightcoral; {BOLD}"
S_TRACK_OFF = f"background-color: lightgray; {BOLD}"
S_MOVE = f"background-color: lightgreen; {BOLD}"
S_STOP = f"background-color: salmon; {BOLD}"
PANEL_QSS = """
QGroupBox { font-weight: bold; border: 1px solid #b0b0b0; border-radius: 5px;
            margin-top: 8px; padding-top: 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { min-height: 28px; padding: 2px 10px; }
QLineEdit { min-height: 22px; }
"""


class ControlsWidget(QWidget):
    paramsChanged = pyqtSignal()
    phiChanged = pyqtSignal(int)
    elbowChanged = pyqtSignal(bool)
    gripperChanged = pyqtSignal(int)
    modeToggled = pyqtSignal(bool)
    addWaypointClicked = pyqtSignal()
    undoWaypointClicked = pyqtSignal()
    clearWaypointsClicked = pyqtSignal()
    moveClicked = pyqtSignal()
    runWaypointsClicked = pyqtSignal()
    stopClicked = pyqtSignal()
    loopToggled = pyqtSignal(bool)
    autoPhiToggled = pyqtSignal(bool)
    trackPathToggled = pyqtSignal(bool)

    def __init__(self, arm, initial_values, parent=None):
        super().__init__(parent)
        self.setStyleSheet(PANEL_QSS)
        self._build_ui(arm, initial_values)

    # ------------------------------------------------------------------
    @staticmethod
    def _field(value, width=64):
        w = QLineEdit(str(value))
        w.setValidator(QDoubleValidator())
        w.setFixedWidth(width)
        w.setAlignment(Qt.AlignRight)
        return w

    @staticmethod
    def _group(title, rows):
        """rows: list of (label_text, widget); laid out as a label/field grid."""
        box = QGroupBox(title)
        grid = QGridLayout(box)
        grid.setContentsMargins(10, 10, 10, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        for r, (text, widget) in enumerate(rows):
            grid.addWidget(QLabel(text), r, 0, Qt.AlignRight)
            grid.addWidget(widget, r, 1)
        return box

    def _build_ui(self, arm, init):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(8)

        # --- Fields -----------------------------------------------------
        self.tb_l1, self.tb_l2, self.tb_l3 = (self._field(v) for v in (arm.L1, arm.L2, arm.L3))
        self.tb_xmin, self.tb_ymin = self._field(arm.x_min), self._field(arm.y_min)
        self.tb_xlim = self._field(init.get('xlim', 400))
        self.tb_ylim = self._field(init.get('ylim', 400))
        self.tb_bez = self._field(init['bezier_weight'])
        self.tb_dur = self._field(init['duration_s'])
        self.tb_dwell = self._field(init['dwell_s'])

        # --- Orientation group (dial + elbow + gripper) -----------------
        self.dial_phi = QDial()
        self.dial_phi.setRange(-180, 180)
        self.dial_phi.setValue(45)
        self.dial_phi.setWrapping(True)
        self.dial_phi.setNotchesVisible(True)
        self.dial_phi.setFixedSize(80, 80)
        self.dial_phi.valueChanged.connect(self._on_phi_dial_changed)

        self.tb_phi = QLineEdit("45")
        self.tb_phi.setFixedWidth(56)
        self.tb_phi.setAlignment(Qt.AlignRight)
        self.tb_phi.editingFinished.connect(self._on_phi_text_changed)

        self.btn_elbow = QPushButton("Elbow: UP")
        self.btn_elbow.setCheckable(True)
        self.btn_elbow.setChecked(True)
        self.btn_elbow.toggled.connect(self._on_elbow_toggled)

        self.slider_gripper = QSlider(Qt.Horizontal)
        self.slider_gripper.valueChanged.connect(self.gripperChanged.emit)

        orient = QGroupBox("Orientation && Gripper")
        og = QGridLayout(orient)
        og.setContentsMargins(10, 10, 10, 8)
        og.setHorizontalSpacing(8)
        og.setVerticalSpacing(6)
        og.addWidget(self.dial_phi, 0, 0, 3, 1)
        og.addWidget(QLabel("Phi (°):"), 0, 1, Qt.AlignRight)
        og.addWidget(self.tb_phi, 0, 2)
        og.addWidget(self.btn_elbow, 1, 1, 1, 2)
        og.addWidget(QLabel("Gripper:"), 2, 1, Qt.AlignRight)
        og.addWidget(self.slider_gripper, 2, 2)
        og.setColumnMinimumWidth(2, 110)

        # --- Parameter row ---------------------------------------------
        top = QHBoxLayout()
        top.setSpacing(10)
        top.addWidget(self._group("Link Lengths", [("L1:", self.tb_l1), ("L2:", self.tb_l2), ("L3:", self.tb_l3)]))
        top.addWidget(self._group("Workspace Limits", [("Min X:", self.tb_xmin), ("Min Y:", self.tb_ymin)]))
        top.addWidget(self._group("Plot Range", [("X lim:", self.tb_xlim), ("Y lim:", self.tb_ylim)]))
        top.addWidget(self._group("Motion", [("Bezier W:", self.tb_bez), ("Duration (s):", self.tb_dur), ("Dwell (s):", self.tb_dwell)]))
        top.addWidget(orient)
        top.addStretch(1)
        outer.addLayout(top)

        for w in (self.tb_l1, self.tb_l2, self.tb_l3, self.tb_xmin, self.tb_ymin,
                  self.tb_xlim, self.tb_ylim, self.tb_bez, self.tb_dur, self.tb_dwell):
            w.editingFinished.connect(self.paramsChanged.emit)

        # --- Action row ------------------------------------------------
        self.btn_mode = self._btn("Mode: OFFLINE", S_MODE_OFF, checkable=True)
        self.btn_mode.toggled.connect(self.modeToggled.emit)

        self.btn_add_wp = self._btn("Add WP", sig=self.addWaypointClicked)
        self.btn_undo_wp = self._btn("Undo WP", sig=self.undoWaypointClicked)
        self.btn_clear_wp = self._btn("Clear WPs", sig=self.clearWaypointsClicked)
        self.btn_move = self._btn("Move Arm", S_MOVE, sig=self.moveClicked)
        self.btn_run_wp = self._btn("Run WPs", sig=self.runWaypointsClicked)
        self.btn_stop = self._btn("Stop", S_STOP, sig=self.stopClicked)

        self.btn_loop = self._btn("Loop: OFF", checkable=True)
        self.btn_loop.toggled.connect(self._on_loop_toggled)

        self.btn_auto_phi = self._btn("Auto Phi: ON", S_AUTO_ON, checkable=True, checked=True)
        self.btn_auto_phi.toggled.connect(self._on_auto_phi_toggled)

        self.btn_track = self._btn("Track: ON", S_TRACK_ON, checkable=True, checked=True)
        self.btn_track.toggled.connect(self._on_track_toggled)

        def row(title, buttons):
            box = QGroupBox(title)
            lay = QHBoxLayout(box)
            lay.setContentsMargins(10, 10, 10, 8)
            lay.setSpacing(8)
            for b in buttons:
                lay.addWidget(b)
            return box

        actions = QHBoxLayout()
        actions.setSpacing(10)
        actions.addWidget(row("Connection", [self.btn_mode]))
        actions.addWidget(row("Waypoints", [self.btn_add_wp, self.btn_undo_wp, self.btn_clear_wp]))
        actions.addWidget(row("Execution", [self.btn_move, self.btn_run_wp, self.btn_stop, self.btn_loop]))
        actions.addWidget(row("Options", [self.btn_auto_phi, self.btn_track]))
        actions.addStretch(1)
        outer.addLayout(actions)

        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

    @staticmethod
    def _btn(text, style="", checkable=False, checked=False, sig=None):
        b = QPushButton(text)
        if style:
            b.setStyleSheet(style)
        if checkable:
            b.setCheckable(True)
            b.setChecked(checked)
        if sig is not None:
            b.clicked.connect(lambda _=False, s=sig: s.emit())
        return b

    # ------------------------------------------------------------------
    def _set_phi_widgets(self, ival):
        self.dial_phi.blockSignals(True)
        self.dial_phi.setValue(ival)
        self.dial_phi.blockSignals(False)
        self.tb_phi.setText(str(ival))

    def _on_phi_dial_changed(self, val):
        self.tb_phi.setText(str(val))
        self.phiChanged.emit(val)

    def _on_phi_text_changed(self):
        try:
            val = float(self.tb_phi.text().strip())
        except ValueError:
            self.tb_phi.setText(str(self.dial_phi.value()))
            return
        ival = int(round(((val + 180.0) % 360.0) - 180.0))
        self._set_phi_widgets(ival)
        self.phiChanged.emit(ival)

    def _on_elbow_toggled(self, checked):
        self.btn_elbow.setText("Elbow: UP" if checked else "Elbow: DOWN")
        self.elbowChanged.emit(checked)

    def _on_loop_toggled(self, checked):
        self.btn_loop.setText(f"Loop: {'ON' if checked else 'OFF'}")
        self.loopToggled.emit(checked)

    def _on_auto_phi_toggled(self, checked):
        self.btn_auto_phi.setText(f"Auto Phi: {'ON' if checked else 'OFF'}")
        self.btn_auto_phi.setStyleSheet(S_AUTO_ON if checked else S_AUTO_OFF)
        self.autoPhiToggled.emit(checked)

    def _on_track_toggled(self, checked):
        self.btn_track.setText(f"Track: {'ON' if checked else 'OFF'}")
        self.btn_track.setStyleSheet(S_TRACK_ON if checked else S_TRACK_OFF)
        self.trackPathToggled.emit(checked)

    # ---- Silent setters (no signals emitted) --------------------------
    def set_phi_silently(self, phi_deg):
        self._set_phi_widgets(int(round(phi_deg)))

    def set_elbow_silently(self, elbow_up):
        self.btn_elbow.blockSignals(True)
        self.btn_elbow.setChecked(elbow_up)
        self.btn_elbow.setText("Elbow: UP" if elbow_up else "Elbow: DOWN")
        self.btn_elbow.blockSignals(False)

    def set_gripper_range(self, lo, hi):
        self.slider_gripper.blockSignals(True)
        self.slider_gripper.setRange(int(lo), int(hi))
        self.slider_gripper.blockSignals(False)

    def set_gripper_value(self, val):
        self.slider_gripper.blockSignals(True)
        self.slider_gripper.setValue(int(val))
        self.slider_gripper.blockSignals(False)

    def set_mode_button(self, online):
        self.btn_mode.blockSignals(True)
        self.btn_mode.setChecked(online)
        self.btn_mode.setText("Mode: ONLINE" if online else "Mode: OFFLINE")
        self.btn_mode.setStyleSheet(S_MODE_ON if online else S_MODE_OFF)
        self.btn_mode.blockSignals(False)

    def revert_mode_button(self):
        self.btn_mode.blockSignals(True)
        self.btn_mode.setChecked(not self.btn_mode.isChecked())
        self.btn_mode.blockSignals(False)
