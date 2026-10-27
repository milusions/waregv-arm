"""Center-spinner overlay with snapshot-based blur / dim."""
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QPixmap, QImage
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout,
    QGraphicsBlurEffect, QGraphicsScene, QGraphicsPixmapItem,
)


def _gaussian_blur(pixmap: QPixmap, radius: int = 14) -> QPixmap:
    """Real gaussian blur using QGraphicsBlurEffect rendered offscreen."""
    scene = QGraphicsScene()
    item = QGraphicsPixmapItem(pixmap)
    effect = QGraphicsBlurEffect()
    effect.setBlurRadius(radius)
    item.setGraphicsEffect(effect)
    scene.addItem(item)

    out = QImage(pixmap.size(), QImage.Format_ARGB32)
    out.fill(Qt.transparent)
    p = QPainter(out)
    p.setRenderHint(QPainter.Antialiasing)
    scene.render(p, QRectF(out.rect()), QRectF(pixmap.rect()))
    p.end()
    return QPixmap.fromImage(out)


class LoadingOverlay(QWidget):
    """Snapshot-based overlay.

    'blur' : frozen blurred snapshot + dark tint + spinner (blocks input)
    'dim'  : plain translucent tint + spinner        (plot stays live)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_NoSystemBackground)
        self.setVisible(False)

        self._mode = 'dim'
        self._angle = 0
        self._message = "Working..."
        self._snapshot = None            # QPixmap for 'blur' mode

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setAlignment(Qt.AlignCenter)

        self.lbl = QLabel(self._message)
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setStyleSheet(
            "color: white; font-size: 18px; font-weight: bold;"
            "background: rgba(0,0,0,150); padding: 14px 26px;"
            "border-radius: 10px;"
        )
        lay.addWidget(self.lbl, alignment=Qt.AlignCenter)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._spin)
        self._timer.setInterval(16)

    # ------------------------------------------------------------------
    def _spin(self):
        self._angle = (self._angle + 8) % 360
        self.update()

    def start(self, message="Working...", mode='dim'):
        self._mode = mode
        self._message = message
        self.lbl.setText(message)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, mode != 'blur')

        if mode == 'blur':
            self._snapshot = self._grab_snapshot()
        else:
            self._snapshot = None

        self._raise_geometry()
        self.show()
        self.raise_()
        self._timer.start()
        
        # Force an immediate paint so the loader is visible before the 
        # main thread blocks for IK calculation.
        self.repaint()

    def stop(self):
        """Hides immediately."""
        self._timer.stop()
        self._snapshot = None
        self.hide()
        self.repaint()

    # ------------------------------------------------------------------
    def _grab_snapshot(self):
        parent = self.parentWidget()
        if parent is None:
            return None
        pm = parent.grab()
        return _gaussian_blur(pm, radius=14)

    def _raise_geometry(self):
        parent = self.parentWidget()
        if parent is not None:
            self.setGeometry(parent.rect())

    def resizeEvent(self, event):
        self._raise_geometry()
        super().resizeEvent(event)

    # ------------------------------------------------------------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        if self._mode == 'blur' and self._snapshot is not None:
            p.drawPixmap(0, 0, self._snapshot)
            p.fillRect(self.rect(), QColor(0, 0, 0, 140))
        else:
            p.fillRect(self.rect(), QColor(0, 0, 0, 60))

        center = self.rect().center()
        size = 90
        arc = QRectF(center.x() - size / 2,
                     center.y() - size / 2 - 40,
                     size, size)

        pen = QPen(QColor(255, 255, 255, 70))
        pen.setWidth(7)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        p.drawArc(arc, 0, 360 * 16)

        pen.setColor(QColor(90, 170, 255))
        p.setPen(pen)
        p.drawArc(arc, -self._angle * 16, 100 * 16)
        p.end()