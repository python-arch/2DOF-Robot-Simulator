from math import *
from Point import Point

delta = 1e-6
class Robot:
    origin = Point(0, 0)
    shoulder_length = 150
    elbow_length = 120
    mergeWristElbow = False

    dynamicMode = True
    approachFromUp = True
    wristApproachAngle = 40
    mappedApproachPair = (120, shoulder_length / 2)

    def handlePoint(point):
        # Ensure the point is within the arm's maximum reach
        maxReach = Robot.shoulder_length + Robot.elbow_length - delta
        if point.magnitude() > maxReach:
            point.changeMagnitude(maxReach)
        return point

    def handle_negative_beta(phi, theta):
        # Choose between the two possible IK solutions based on approachFromUp
        return phi - theta if (phi + theta < phi - theta) ^ Robot.approachFromUp else phi + theta

    def inverseKinematics(point):
        point = Robot.handlePoint(point)
        
        # Convert cartesian coordinates to polar form
        a = point.x
        z = point.y
        phi = atan2(z, a)
        L = sqrt(a**2 + z**2)
        
        # Calculate joint angles using law of cosines
        cos_theta = (Robot.shoulder_length**2 + L**2 - Robot.elbow_length**2) / (2 * Robot.shoulder_length * L)
        theta = acos(cos_theta)
        
        # Calculate first motor angle (shoulder)
        beta_1 = Robot.handle_negative_beta(phi, theta)
        
        # Calculate second motor angle (elbow)
        cos_beta_2 = (Robot.shoulder_length**2 + Robot.elbow_length**2 - L**2) / (2 * Robot.shoulder_length * Robot.elbow_length)
        beta_2 = acos(cos_beta_2)
        
        # Convert to motor angles
        motor1 = beta_1
        motor2 = pi - beta_2 if Robot.approachFromUp else beta_2 - 2 * pi
        
        return motor1, motor2