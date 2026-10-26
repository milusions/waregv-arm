import sys
from PyQt5.QtWidgets import QApplication
import app.app_qt  as app

def main():
    print("Starting 3-DOF Planar Robotic Arm PyQt Application...")
    print("Loading parameters from config.yaml...")
    app.main()

if __name__ == "__main__":
    main()