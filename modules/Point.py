from math import *

class Point:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def copy(point):
        return Point(point.x, point.y)

    def magnitude(self):
        return sqrt(self.x**2 + self.y**2)

    def changeMagnitude(self, newMagnitude):
        oldMagnitude = self.magnitude()
        self.x *= newMagnitude / oldMagnitude
        self.y *= newMagnitude / oldMagnitude

    def apply(self, func):
        self.x = func(self.x)
        self.y = func(self.y)
        return self