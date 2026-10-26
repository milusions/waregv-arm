"""Matplotlib canvas widget: main arm plot, ghost preview, joint history graphs."""
import math
from bisect import bisect_left

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class ArmPlotWidget(QWidget):
    """Main arm plot (left), 3 joint history graphs (right), ghost preview."""

    JOINT_COLORS = ['tab:blue', 'tab:green', 'tab:orange']
    GRAPH_WINDOW_S = 20.0

    def __init__(self, arm, parent=None):
        super().__init__(parent)
        self.arm = arm
        self.max_reach = arm.L1 + arm.L2 + arm.L3
        self.custom_xlim = self.max_reach + 20
        self.custom_ylim = self.max_reach + 20

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Figure used directly (no pyplot global state -> lighter, no leaks)
        self.fig = Figure(figsize=(12, 6))
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax = self.fig.add_axes([0.06, 0.10, 0.54, 0.80])

        self.graph_axes, self.graph_lines = [], []
        self.limit_lines, self.target_lines = [], []
        for i in range(3):
            ax = self.fig.add_axes([0.69, 0.71 - i * 0.30, 0.29, 0.19])
            line, = ax.plot([], [], color=self.JOINT_COLORS[i], linewidth=1.8)
            lo = ax.axhline(0, color='red', linestyle='--', linewidth=1)
            hi = ax.axhline(0, color='red', linestyle='--', linewidth=1)
            tgt = ax.axhline(0, color='purple', linestyle=':', linewidth=1.8,
                             alpha=0.85, zorder=3)
            ax.grid(True, alpha=0.4)
            ax.set_ylabel(f"θ{i + 1} (deg)")
            ax.tick_params(labelsize=8)
            self.graph_axes.append(ax)
            self.graph_lines.append(line)
            self.limit_lines.append((lo, hi))
            self.target_lines.append(tgt)

        self.arm_line = self.base_point = self.target_point = None
        self.wp_line = self.wp_points = self.trace_line = None
        self.wp_texts = []
        self.ghost_arm_line = self.ghost_target_point = None
        self.ghost_visible = False
        self._last_xlim = None

        self.setup_plot()

    # ------------------------------------------------------------------
    def set_plot_limits(self, xlim, ylim):
        self.custom_xlim, self.custom_ylim = xlim, ylim
        self.setup_plot()

    def setup_plot(self):
        ax, arm = self.ax, self.arm
        ax.clear()
        self.max_reach = arm.L1 + arm.L2 + arm.L3
        span = self.max_reach * 2

        ax.set_xlim(min(-20, arm.x_min - 20), self.custom_xlim)
        ax.set_ylim(min(-self.custom_ylim, arm.y_min - 20), self.custom_ylim)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.axhline(0, color='black', linewidth=1)
        ax.axvline(arm.x_min, color='black', linewidth=2)
        ax.axhline(arm.y_min, color='black', linewidth=2)
        ax.axvspan(-span, arm.x_min, color='red', alpha=0.1, zorder=0)
        ax.axhspan(-span, arm.y_min, color='red', alpha=0.1, zorder=0)
        ax.set_title("Click to set target, then 'Move Arm'", fontsize=10)

        self.ghost_arm_line, = ax.plot([], [], '-o', linewidth=4, markersize=8,
                                       color='gray', alpha=0.45, zorder=1)
        self.ghost_target_point, = ax.plot([], [], 'x', markersize=11,
                                           markeredgewidth=2, color='gray',
                                           alpha=0.7, zorder=2)
        self.ghost_visible = False

        self.wp_line, = ax.plot([], [], '--', color='green', linewidth=1.2, zorder=2)
        self.wp_points, = ax.plot([], [], 'o', color='green', markersize=9, zorder=3)
        self.trace_line, = ax.plot([], [], '-', color='darkorange',
                                   linewidth=1.5, zorder=2)
        self.arm_line, = ax.plot([], [], '-o', linewidth=5, markersize=10,
                                 color='royalblue')
        self.base_point, = ax.plot([0], [0], 'ks', markersize=12)
        self.target_point, = ax.plot([], [], 'rX', markersize=12)

        self.wp_texts = []
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    def _fk(self, t1, t2, t3):
        a = self.arm
        c1, s1 = math.cos(t1), math.sin(t1)
        x1, y1 = a.L1 * c1, a.L1 * s1
        t12 = t1 + t2
        x2, y2 = x1 + a.L2 * math.cos(t12), y1 + a.L2 * math.sin(t12)
        t123 = t12 + t3
        x3, y3 = x2 + a.L3 * math.cos(t123), y2 + a.L3 * math.sin(t123)
        return [0, x1, x2, x3], [0, y1, y2, y3]

    def draw_arm(self, t1, t2, t3):
        self.arm_line.set_data(*self._fk(t1, t2, t3))

    def draw_ghost_arm(self, t1, t2, t3):
        self.ghost_arm_line.set_data(*self._fk(t1, t2, t3))
        self.ghost_visible = True

    def clear_ghost_arm(self):
        self.ghost_arm_line.set_data([], [])
        self.ghost_target_point.set_data([], [])
        self.ghost_visible = False

    def show_ghost_target(self, x, y):
        self.ghost_target_point.set_data([x], [y])

    def set_target_marker(self, x, y, valid=True):
        self.target_point.set_data([x], [y])
        self.target_point.set_color('red' if valid else 'gray')

    def set_trace(self, points):
        if not points:
            self.trace_line.set_data([], [])
        else:
            xs, ys = zip(*points)
            self.trace_line.set_data(xs, ys)

    def redraw_waypoints(self, waypoints, draw=True):
        xs = [w['x'] for w in waypoints]
        ys = [w['y'] for w in waypoints]
        self.wp_line.set_data(xs, ys)
        self.wp_points.set_data(xs, ys)
        for t in self.wp_texts:
            t.remove()
        self.wp_texts = [
            self.ax.annotate(str(i + 1), (x, y), textcoords='offset points',
                             xytext=(7, 7), color='darkgreen',
                             fontsize=10, weight='bold')
            for i, (x, y) in enumerate(zip(xs, ys))
        ]
        if draw:
            self.canvas.draw_idle()

    # ------------------------------------------------------------------
    def update_limit_display(self, joint_limits):
        for i, ax in enumerate(self.graph_axes):
            lo, hi = (math.degrees(v) for v in joint_limits[i])
            lo_line, hi_line = self.limit_lines[i]
            lo_line.set_ydata([lo, lo])
            hi_line.set_ydata([hi, hi])
            pad = max(10.0, 0.1 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
            ax.set_title(f"θ{i + 1}   min {lo:.1f}°   max {hi:.1f}°", fontsize=9)

    def set_target_angles(self, t1_rad, t2_rad, t3_rad, valid=True):
        vals = (t1_rad, t2_rad, t3_rad)
        for line, v in zip(self.target_lines, vals):
            if valid and v is not None:
                deg = math.degrees(v)
                line.set_ydata([deg, deg])
                line.set_visible(True)
            else:
                line.set_visible(False)

    def refresh_graphs(self, hist_t, hist_a, window_s=None):
        """Plot only the visible time window (history can grow unbounded)."""
        if not hist_t:
            return
        window_s = window_s or self.GRAPH_WINDOW_S
        x_min = max(0.0, hist_t[-1] - window_s)
        start = bisect_left(hist_t, x_min)
        t = hist_t[start:]
        xlim = (x_min, x_min + window_s)
        for i, line in enumerate(self.graph_lines):
            line.set_data(t, hist_a[i][start:])
            self.graph_axes[i].set_xlim(*xlim)
