from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QSlider, QCheckBox, QFrame, 
                           QPushButton, QLineEdit, QGridLayout)
import sys
sys.path.append('/Users/python/Desktop/Robotics Bonus/modules')
from Robot import Robot
from Point import Point
from math import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QColor
from PathGenerator import pathGenerator

point_0 = [4, 6]
point_f = [10, -1]
mag_ang_0 = [2, 90]
mag_ang_f = [3, 180]

c = [point_0, None, None, point_f]
c[1] = pathGenerator.tangentPoint(c[0], magnitude=mag_ang_0[0], angle=mag_ang_0[1])
c[2] = pathGenerator.tangentPoint(c[3], magnitude=mag_ang_f[0], angle=mag_ang_f[1])

curve_length = pathGenerator.calculateCurveLength(c)
n = pathGenerator.getNumberOfPoints(curve_length)
numberOfPoints = ceil(n / 8 + 2 * sqrt(n))
pathTimeStamps = pathGenerator.bellSegmenter(60)
Bezier_curve = [pathGenerator.Bezier_curve(timestamp, c) for timestamp in pathTimeStamps]

class RobotArmCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(700, 700)
        self.setBaseSize(700, 700)
        self.grid_size = 30
        self.origin = Point(self.grid_size*4,self.grid_size *13)
        self.endEffectorPosition = Point(0, 0)
    def calculate_intersection(self):
        # Coefficients for the quadratic equation 5x^2 - 40x + 76 = 0
        a = 5
        b = -40
        c = 76

        # Solve using the quadratic formula
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return None  # No real intersection
        
        sqrt_discriminant = sqrt(discriminant)
        x1 = (-b + sqrt_discriminant) / (2 * a)
        x2 = (-b - sqrt_discriminant) / (2 * a)

        # Corresponding y values from the line equation y = -x/2 + 10
        y1 = -x1 / 2 + 10
        y2 = -x2 / 2 + 10

        # Return the intersection points
        return (x1, y1), (x2, y2)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw grid
        painter.setPen(QPen(QColor('lightgray')))
        for i in range(0, 700, self.grid_size):
            painter.drawLine(0, i, 700, i)
            painter.drawLine(i, 0, i, 700)
            
        # Draw arm components
        if hasattr(self, 'arm_components'):
            # Draw base to shoulder
            painter.setPen(QPen(QColor('black'), 7))
            painter.drawLine(self.arm_components['base_x'], self.arm_components['base_y'],
                           self.arm_components['shoulder_x'], self.arm_components['shoulder_y'])
            
            # Draw shoulder to elbow
            painter.setPen(QPen(QColor('black'), 5))
            painter.drawLine(self.arm_components['shoulder_x'], self.arm_components['shoulder_y'],
                           self.arm_components['end_x'], self.arm_components['end_y'])
            
           
            
            # Draw joints
            painter.setPen(QPen(QColor('blue')))
            painter.setBrush(QColor('blue'))
            painter.drawEllipse(self.arm_components['shoulder_x'] - 6, 
                              self.arm_components['shoulder_y'] - 6, 12, 12)
            
        # Draw Bezier curve points
        if hasattr(self, 'bezier_points'):
            painter.setPen(QPen(QColor('red')))
            painter.setBrush(QColor('red'))
            for point in self.bezier_points:
                pixel_point = self.reverse_PixelPoint_to_GridPoint(Point(point[0], point[1]))
                painter.drawEllipse(int(pixel_point.x) - 2, 
                                  int(pixel_point.y) - 2, 4, 4)

    def update_arm(self, components):
        self.arm_components = components
        self.update()

    def set_bezier_points(self, points):
        self.bezier_points = points
        self.update()

    def pixelToGrid(self, x):
        return x / self.grid_size

    def gridToPixel(self, x):
        return x * self.grid_size
    
    def GridPoint_to_PixelPoint_Relative_To_Origin(self, point):
        point.apply(self.gridToPixel)
        point.y = -point.y
        return point

    def PixelPoint_to_GridPoint(self, point):
        gridPoint = Point.copy(point)
        gridPoint.x = self.pixelToGrid(point.x - self.origin.x)
        gridPoint.y = self.pixelToGrid(self.origin.y - point.y)
        return gridPoint

    def reverse_PixelPoint_to_GridPoint(self, point):
        pixelPoint = Point.copy(point)
        pixelPoint.x = self.origin.x + self.gridToPixel(point.x)
        pixelPoint.y = self.origin.y - self.gridToPixel(point.y)
        return pixelPoint

scriptedMoves = [ [False , True , 0] , [False , True , 45] , [False , True , 90]  , [True , True , 90] ]

class RobotArmSimulator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2 DOF Robot Arm Simulator")
        
        # Initialize variables
        self.base_angle = 0
        self.shoulder_angle = 0
        self.manual_mode = True
        self.update_delay = 25
        self.RepeatDelayCount = 30
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        
        # Create canvas
        self.canvas = RobotArmCanvas()
        intersection_points = self.canvas.calculate_intersection()
        if intersection_points:
            print(f"Intersection Points: {intersection_points}")
        layout.addWidget(self.canvas)
        
        
        self.stepTracker = 0
        self.timeCounter = 0
        self.moveindex = 0
        
        # Create control panel
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        layout.addWidget(control_panel)
        
        # Mode selection
        mode_label = QLabel("Mode:")
        control_layout.addWidget(mode_label)
        
        self.mode_switch = QCheckBox("Manual Mode")
        self.mode_switch.setChecked(True)
        self.mode_switch.stateChanged.connect(self.toggle_mode)
        control_layout.addWidget(self.mode_switch)
        
        # Sliders
        slider_frame = QFrame()
        slider_frame.setFrameStyle(QFrame.Box)
        slider_layout = QGridLayout(slider_frame)
        
        # Base angle slider
        base_label = QLabel("Base Angle")
        slider_layout.addWidget(base_label, 0, 0)
        self.base_slider = QSlider(Qt.Horizontal)
        self.base_slider.setRange(-180, 180)
        self.base_slider.valueChanged.connect(self.update_base_angle)
        slider_layout.addWidget(self.base_slider, 0, 1)
        
        # Shoulder angle slider
        shoulder_label = QLabel("Shoulder Angle")
        slider_layout.addWidget(shoulder_label, 1, 0)
        self.shoulder_slider = QSlider(Qt.Horizontal)
        self.shoulder_slider.setRange(-180, 180)
        self.shoulder_slider.valueChanged.connect(self.update_shoulder_angle)
        slider_layout.addWidget(self.shoulder_slider, 1, 1)
        
        
        control_layout.addWidget(slider_frame)
        
        # Target panel
        target_panel = QFrame()
        target_panel.setFrameStyle(QFrame.Box)
        target_layout = QGridLayout(target_panel)
        
        # Initial position inputs
        target_layout.addWidget(QLabel("Initial X:"), 0, 0)
        self.initial_x_entry = QLineEdit()
        target_layout.addWidget(self.initial_x_entry, 0, 1)
        
        target_layout.addWidget(QLabel("Initial Y:"), 1, 0)
        self.initial_y_entry = QLineEdit()
        target_layout.addWidget(self.initial_y_entry, 1, 1)
        
        # Target position inputs
        target_layout.addWidget(QLabel("Target X:"), 0, 2)
        self.target_x_entry = QLineEdit()
        target_layout.addWidget(self.target_x_entry, 0, 3)
        
        target_layout.addWidget(QLabel("Target Y:"), 1, 2)
        self.target_y_entry = QLineEdit()
        target_layout.addWidget(self.target_y_entry, 1, 3)
        
        control_layout.addWidget(target_panel)
        
        # Angle panel
        angle_panel = QFrame()
        angle_panel.setFrameStyle(QFrame.Box)
        angle_layout = QGridLayout(angle_panel)
        
        # Departure angle inputs
        angle_layout.addWidget(QLabel("Departure θ:"), 0, 0)
        self.angle_departure_entry = QLineEdit()
        angle_layout.addWidget(self.angle_departure_entry, 0, 1)
        
        angle_layout.addWidget(QLabel("Departure mag.:"), 1, 0)
        self.magnitude_departure_entry = QLineEdit()
        angle_layout.addWidget(self.magnitude_departure_entry, 1, 1)
        
        # Arrival angle inputs
        angle_layout.addWidget(QLabel("Arrival θ:"), 0, 2)
        self.angle_arrival_entry = QLineEdit()
        angle_layout.addWidget(self.angle_arrival_entry, 0, 3)
        
        angle_layout.addWidget(QLabel("Arrival mag.:"), 1, 2)
        self.magnitude_arrival_entry = QLineEdit()
        angle_layout.addWidget(self.magnitude_arrival_entry, 1, 3)
        
    
        
        # Set target button
        self.set_target_button = QPushButton("Set Target")
        self.set_target_button.clicked.connect(self.set_target)
        angle_layout.addWidget(self.set_target_button, 2, 0, 1, 4)
        
        control_layout.addWidget(angle_panel)
        
        # Position labels
        self.x_label = QLabel("x: 0")
        self.y_label = QLabel("y: 0")
        control_layout.addWidget(self.x_label)
        control_layout.addWidget(self.y_label)
        
        # Initialize timer for automated mode
        self.timer = QTimer()
        self.timer.timeout.connect(self.automatedMode)
        
        # Initial draw
        self.draw_arm()

    def toggle_mode(self):
        self.manual_mode = self.mode_switch.isChecked()
        if self.manual_mode:
            self.timer.stop()
            self.base_slider.setEnabled(True)
            self.shoulder_slider.setEnabled(True)
        else:
            self.stepTracker = 0
            self.timer.start(self.update_delay)
            self.base_slider.setEnabled(False)
            self.shoulder_slider.setEnabled(False)

    def automatedMode(self):
        if not self.manual_mode:
            if self.stepTracker != len(Bezier_curve) - 1:
                self.stepTracker += 1
                self.timeCounter = 0
            else:
                self.timeCounter += 1
                if self.timeCounter == self.RepeatDelayCount:
                    self.stepTracker = 0
                    Robot.approachFromUp = True  # Simplified for 2-DOF
                    self.moveindex += 1
                    if self.moveindex == len(scriptedMoves):
                        self.moveindex = 0
            
            targetPoint = Point(Bezier_curve[self.stepTracker][0], Bezier_curve[self.stepTracker][1])
            pixelTarget = self.canvas.GridPoint_to_PixelPoint_Relative_To_Origin(targetPoint)
            self.base_angle, self.shoulder_angle = [
                degrees(angle) for angle in Robot.inverseKinematics(pixelTarget)
            ]
            self.base_slider.setValue(self.base_angle)
            self.shoulder_slider.setValue(self.shoulder_angle)
            self.draw_arm()

    def update_base_angle(self, angle):
        if self.manual_mode:
            self.base_angle = angle
            self.draw_arm()

    def update_shoulder_angle(self, angle):
        if self.manual_mode:
            self.shoulder_angle = angle
            self.draw_arm()

    def draw_arm(self):
        base = self.canvas.reverse_PixelPoint_to_GridPoint(Robot.origin)
        base_x = base.x
        base_y = base.y
        
        shoulder_length = Robot.shoulder_length
        elbow_length = Robot.elbow_length
        
        shoulder_x = base_x + shoulder_length * cos(radians(self.base_angle))
        shoulder_y = base_y + shoulder_length * sin(radians(self.base_angle))
        
        
        end_x = shoulder_x + elbow_length * cos(radians(self.base_angle + self.shoulder_angle))
        end_y = shoulder_y + elbow_length * sin(radians(self.base_angle + self.shoulder_angle))
        
        components = {
            'base_x': base_x,
            'base_y': base_y,
            'shoulder_x': shoulder_x,
            'shoulder_y': shoulder_y,
            'end_x': end_x,
            'end_y': end_y
        }
        
        self.canvas.update_arm(components)
        
        endEffector = Point(end_x, end_y)
        self.canvas.endEffectorPosition = self.canvas.PixelPoint_to_GridPoint(endEffector)
        
        self.x_label.setText(f"x: {round(self.canvas.endEffectorPosition.x, 2)}")
        self.y_label.setText(f"y: {round(self.canvas.endEffectorPosition.y, 2)}")

    def set_target(self):
        try:
            initial_x = float(self.initial_x_entry.text())
            initial_y = float(self.initial_y_entry.text())
            target_x = float(self.target_x_entry.text())
            target_y = float(self.target_y_entry.text())
            dep_angle = float(self.angle_departure_entry.text())
            dep_magnitude = float(self.magnitude_departure_entry.text())
            arr_angle = float(self.angle_arrival_entry.text())
            arr_magnitude = float(self.magnitude_arrival_entry.text())
            
            # Define control points
            point_0 = [initial_x, initial_y]
            point_f = [target_x, target_y]
            mag_ang_0 = [dep_magnitude, dep_angle]
            mag_ang_f = [arr_magnitude, arr_angle]

            c = [point_0, None, None, point_f]
            c[1] = pathGenerator.tangentPoint(c[0], magnitude=mag_ang_0[0], angle=mag_ang_0[1])
            c[2] = pathGenerator.tangentPoint(c[3], magnitude=mag_ang_f[0], angle=mag_ang_f[1])

            # Generate new path
            curve_length = pathGenerator.calculateCurveLength(c)
            n = pathGenerator.getNumberOfPoints(curve_length)
            numberOfPoints = ceil(n / 8 + 2 * sqrt(n))
            pathTimeStamps = pathGenerator.bellSegmenter(60)
            global Bezier_curve
            Bezier_curve = [pathGenerator.Bezier_curve(timestamp, c) for timestamp in pathTimeStamps]

            # Set new Bezier points on canvas
            self.canvas.set_bezier_points(Bezier_curve)
        except ValueError:
                print("Invalid input for target or angle values. Please enter numeric values.")

    def calculate_bezier_curve(self, start, end, departure_theta, departure_mag, arrival_theta, arrival_mag):
        control1 = Point(start.x + departure_mag * cos(radians(departure_theta)),
                         start.y + departure_mag * sin(radians(departure_theta)))
        
        control2 = Point(end.x - arrival_mag * cos(radians(arrival_theta)),
                         end.y - arrival_mag * sin(radians(arrival_theta)))
        
        num_points = 50  # Number of points on the Bezier curve
        bezier_points = []
        for t in range(num_points + 1):
            t /= num_points
            x = (1 - t) ** 3 * start.x + 3 * (1 - t) ** 2 * t * control1.x + 3 * (1 - t) * t ** 2 * control2.x + t ** 3 * end.x
            y = (1 - t) ** 3 * start.y + 3 * (1 - t) ** 2 * t * control1.y + 3 * (1 - t) * t ** 2 * control2.y + t ** 3 * end.y
            bezier_points.append((x, y))
        
        self.canvas.set_bezier_points(bezier_points)