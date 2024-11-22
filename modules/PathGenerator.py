from math import *
import matplotlib.pyplot as plt


class pathGenerator():
    unitLength = 0.2

    def tangentPoint(p_0, magnitude, angle):
        p_f = list(p_0)
        p_f[0] += magnitude * cos(pi * angle / 180)
        p_f[1] += magnitude * sin(pi * angle / 180)
        return p_f

    def Bezier_curve(t, c):
        B = [0, 0]
        for j in range(2):
            for i in range(4):
                B[j] += comb(3, i) * c[i][j] * (1 - t)**(3 - i) * t**i
        # B = [3*sin(t * 2 * pi)+5 , 3*cos(t * 2 * pi)+5]
        return B

    def calculateCurveLength(c):
        length = 0
        lastPoint = pathGenerator.Bezier_curve(0, c)
        for i in range(1, 1001):
            point = pathGenerator.Bezier_curve(i / 1000, c)
            length += sqrt((point[0] - lastPoint[0])**2 + (point[1] - lastPoint[1])**2)
            lastPoint = point
        return length

    def getNumberOfPoints(l):
        return ceil(l / pathGenerator.unitLength)

    def segmentFunction(x):
        return (1 - cos(2 * pi * x)) / 2

    def bellSegmenter(pointsPerPath=100):
        pointsPerPath = max(1, pointsPerPath + 1)
        timestamps = [pathGenerator.segmentFunction(1 / (pointsPerPath + 1))]
        accumlator = timestamps[0]
        for i in range(2, pointsPerPath):
            timestamp = pathGenerator.segmentFunction(i / (pointsPerPath + 1))
            timestamps.append(timestamps[-1] + timestamp)
            accumlator += timestamp
        timestamps = [timestamp / accumlator for timestamp in timestamps]
        return timestamps

    def graph1DList(data):
        y_values = [0] * len(data)
        plt.scatter(data, y_values)
        plt.ylim(-1, 1)
        plt.yticks([])
        plt.show()

    def graph2DList(data):
        x_values, y_values = [[point[j] for point in data] for j in range(2)]
        plt.scatter(x_values, y_values)
        plt.grid(True)
        plt.show()
