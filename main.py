import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QSlider, QCheckBox, QFrame, 
                           QPushButton, QLineEdit, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
from math import *
import numpy as np
import matplotlib.pyplot as plt
from modules.utils import *
from modules.Point import Point
from modules.PathGenerator import pathGenerator
from modules.RobotArmSimulator import RobotArmSimulator


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotArmSimulator()
    window.show()
    sys.exit(app.exec_())