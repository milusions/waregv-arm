"""Entry point for the robotic arm PyQt controller."""
import sys

from PyQt5.QtWidgets import QApplication

from app.controller import ArmControllerWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = ArmControllerWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
