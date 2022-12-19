#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Aug 03 16:52:07 2022
@author: krzysztof_skos
"""
from math import sqrt
import numpy as np
import numpy.linalg


class Sensor:
    """This class represents a nano sensor (magnetometer) operating in magnetic field

    :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
    :type [ParamName]: [ParamType](, optional)
    ...
    :raises [ErrorType]: [ErrorDescription]
    ...
    :return: [ReturnDescription]
    :rtype: [ReturnType]
    """
    position = (0.0, 0.0, 0.0)  # Position (x, y, z)
    distance = [0.0, 0.0, 0.0]  # Distance calculated
    distanceX = [0.0, 0.0, 0.0, 0.0, 0.0]
    distanceY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    distanceZ = [0.0, 0.0, 0.0, 0.0]
    flux = [0.0, 0.0, 0.0]
    fluxX = [0.0, 0.0, 0.0, 0.0, 0.0]
    fluxY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    fluxZ = [0.0, 0.0, 0.0, 0.0]
    positionEstimated = [0.0, 0.0, 0.0]
    distanceEstimated = [0.0, 0.0, 0.0]
    distanceEstimatedX = [0.0, 0.0, 0.0, 0.0, 0.0]
    distanceEstimatedY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    distanceEstimatedZ = [0.0, 0.0, 0.0, 0.0]
    positionError = [0.0, 0.0, 0.0]
    totalPositionError = 0.0

    def __init__(self, pos):
        """
        Sensor constructor
        :param pos: Sensor position in 3d Cartesian coordinate system. Value in cm
        """
        self.position = pos

    def setDistance(self, dist):
        """
        Distance setter.
        :param dist: Distance between magnet and sensor. Value in ?
        """
        self.distance[0] = dist[0]
        self.distance[1] = dist[1]
        self.distance[2] = dist[2]

    def setDistance15(self, dist):
        self.distanceX[0] = dist[0]
        self.distanceX[1] = dist[1]
        self.distanceX[2] = dist[2]
        self.distanceX[3] = dist[3]
        self.distanceX[4] = dist[4]

        self.distanceY[0] = dist[5]
        self.distanceY[1] = dist[6]
        self.distanceY[2] = dist[7]
        self.distanceY[3] = dist[8]
        self.distanceY[4] = dist[9]
        self.distanceY[5] = dist[10]

        self.distanceZ[0] = dist[11]
        self.distanceZ[1] = dist[12]
        self.distanceZ[2] = dist[13]
        self.distanceZ[3] = dist[14]

    def setFlux(self, flux):
        """
        Flux setter.
        :param flux: flux measured by sensor. Value in T
        """
        self.flux[0] = flux[0]
        self.flux[1] = flux[1]
        self.flux[2] = flux[2]

    def setFlux15(self, flux):
        self.fluxX[0] = flux[0]
        self.fluxX[1] = flux[1]
        self.fluxX[2] = flux[2]
        self.fluxX[3] = flux[3]
        self.fluxX[4] = flux[4]

        self.fluxY[0] = flux[5]
        self.fluxY[1] = flux[6]
        self.fluxY[2] = flux[7]
        self.fluxY[3] = flux[8]
        self.fluxY[4] = flux[9]
        self.fluxY[5] = flux[10]

        self.fluxZ[0] = flux[11]
        self.fluxZ[1] = flux[12]
        self.fluxZ[2] = flux[13]
        self.fluxZ[3] = flux[14]




    def calculateEstimatedDistance15(self, current):
        for i in range(len(self.fluxX)):
            self.distanceEstimatedX[i] = current[0] * 2 * 10 ** (-7) / self.fluxX[i] * 100
        for i in range(len(self.fluxY)):
            self.distanceEstimatedY[i] = current[0] * 2 * 10 ** (-7) / self.fluxY[i] * 100
        for i in range(len(self.fluxZ)):
            self.distanceEstimatedZ[i] = current[0] * 2 * 10 ** (-7) / self.fluxZ[i] * 100

    def calculateEstimatedDistance(self, current):
        """
        This method estimates distance from measured flux. Output is in cm
        """
        self.distanceEstimated[0] = current[0] * 2 * 10 ** (-7) / self.flux[0] * 100  # *100 to change unit from m to cm
        self.distanceEstimated[1] = current[1] * 2 * 10 ** (-7) / self.flux[1] * 100  # *100 to change unit from m to cm
        self.distanceEstimated[2] = current[2] * 2 * 10 ** (-7) / self.flux[2] * 100  # *100 to change unit from m to cm

    def calculateEstimatedPosition15_X(self):
        y1 = 242.0 #100.0
        z1 = 171.0 #100.0
        z2 = 342.0 #200.0
        A = [[y1, 0.0], # y1, 0
             [0.0, z1], # 0, z1
             [0.0, z2], # 0, z2
             [y1, z2]] # y1, z2
        # A = [[100.0, 0.0, 0.0, 100.0],
        #      [0.0, 100.0, 200.0, 200.0]]
        B = [(self.distanceEstimatedX[0] ** 2 - self.distanceEstimatedX[1] ** 2 + y1 ** 2) / 2,
             (self.distanceEstimatedX[0] ** 2 - self.distanceEstimatedX[2] ** 2 + z1 ** 2) / 2,
             (self.distanceEstimatedX[0] ** 2 - self.distanceEstimatedX[3] ** 2 + z2 ** 2) / 2,
             (self.distanceEstimatedX[0] ** 2 - self.distanceEstimatedX[4] ** 2 + y1 ** 2 + z2 ** 2) / 2]
        A = np.array(A)
        B = np.array(B)
        # R = (np.transpose(A) * A) ** (-1) * np.transpose(A) * B
        # R = np.matmul(np.matmul((np.matmul(np.transpose(A), A)) ** (-1), np.transpose(A)), B)
        R = np.matmul(np.matmul(np.linalg.inv((np.matmul(np.transpose(A), A))), np.transpose(A)), B)

        return R

    def calculateEstimatedPosition15_Y(self):
        x1 = 242.0 #100.0
        z1 = 171.0 #100.0
        z2 = 342.0 #200.0
        A = [[x1, 0.0],  # x1, 0
             [0.0, z1],  # 0, z1
             [x1, z1],  # x1, z1
             [0.0, z2],  # 0, z2
             [x1, z2]]  # x1, z2
        B = [(self.distanceEstimatedY[0] ** 2 - self.distanceEstimatedY[1] ** 2 + x1 ** 2) / 2,
             (self.distanceEstimatedY[0] ** 2 - self.distanceEstimatedY[2] ** 2 + z1 ** 2) / 2,
             (self.distanceEstimatedY[0] ** 2 - self.distanceEstimatedY[3] ** 2 + x1 ** 2 + z1 ** 2) / 2,
             (self.distanceEstimatedY[0] ** 2 - self.distanceEstimatedY[4] ** 2 + z2 ** 2) / 2,
             (self.distanceEstimatedY[0] ** 2 - self.distanceEstimatedY[5] ** 2 + x1 ** 2 + z2 ** 2) / 2]
        A = np.array(A)
        B = np.array(B)
        # R = (np.transpose(A) * A) ** (-1) * np.transpose(A) * B
        # R = np.matmul(np.matmul((np.matmul(np.transpose(A), A)) ** (-1), np.transpose(A)), B)
        R = np.matmul(np.matmul(np.linalg.inv((np.matmul(np.transpose(A), A))), np.transpose(A)), B)

        return R

    def calculateEstimatedPosition15_Z(self):
        x1 = 242.0 #100.0
        y1 = 242.0 #100.0
        A = [[x1, 0.0],  # x1, 0
             [0.0, y1],  # 0, y1
             [x1, y1]]  # x1, y1
        B = [(self.distanceEstimatedZ[0] ** 2 - self.distanceEstimatedZ[1] ** 2 + x1 ** 2) / 2,
             (self.distanceEstimatedZ[0] ** 2 - self.distanceEstimatedZ[2] ** 2 + y1 ** 2) / 2,
             (self.distanceEstimatedZ[0] ** 2 - self.distanceEstimatedZ[3] ** 2 + x1 ** 2 + y1 ** 2) / 2]
        A = np.array(A)
        B = np.array(B)
        R = np.matmul(np.matmul(np.linalg.inv((np.matmul(np.transpose(A), A))), np.transpose(A)), B)
        return R

    def calculateEstimatedPosition15(self):
        y1, z1 = self.calculateEstimatedPosition15_X() # weight 5
        x1, z2 = self.calculateEstimatedPosition15_Y() # weight 6
        x2, y2 = self.calculateEstimatedPosition15_Z() # weight 4

        self.positionEstimated[0] = (6 * x1 + 4 * x2) / 10
        self.positionEstimated[1] = (5 * y1 + 4 * y2) / 9
        self.positionEstimated[2] = (6 * z1 + 5 * z2) / 11

        # print("X")
        # print((self.position[0], x1, x2))
        # print("Y")
        # print((self.position[1], y1, y2))
        # print("Z")
        # print((self.position[2], z1, z2))
        # print("Errors")
        # print((abs(self.position[0]-self.positionEstimated[0]), abs(self.position[1]-self.positionEstimated[1]), abs(self.position[2]-self.positionEstimated[2])))
        # print("=======================================")

    def calculateEstimatedPosition(self):
        """
        This method estimates the sensor position from estimated distances. Output is in cm
        """
        # print((self.distanceEstimated[0], self.distanceEstimated[1], self.distanceEstimated[2]))
        try:
            self.positionEstimated[0] = sqrt((self.distanceEstimated[1] ** 2 + self.distanceEstimated[2] ** 2 -
                                              self.distanceEstimated[0] ** 2)  / 2)
        except:
            self.positionEstimated[0] = None
        try:
            self.positionEstimated[1] = sqrt((self.distanceEstimated[0] ** 2 + self.distanceEstimated[2] ** 2 -
                                              self.distanceEstimated[1] ** 2) / 2)
        except:
            self.positionEstimated[1] = None
        try:
            self.positionEstimated[2] = sqrt((self.distanceEstimated[0] ** 2 + self.distanceEstimated[1] ** 2 -
                                              self.distanceEstimated[2] ** 2) / 2)
        except:
            self.positionEstimated[2] = None
        # print ((self.positionEstimated[0], self.positionEstimated[1], self.positionEstimated[2]))

    def calculatePositionError(self):
        """
        This method calculates position error for each sensor coordinate. Output is in cm
        """
        try:
            self.positionError[0] = abs(self.position[0] - self.positionEstimated[0])
        except:
            self.positionError[0] = None
        try:
            self.positionError[1] = abs(self.position[1] - self.positionEstimated[1])
        except:
            self.positionError[1] = None
        try:
            self.positionError[2] = abs(self.position[2] - self.positionEstimated[2])
        except:
            self.positionError[2] = None
        self.calculateTotalPositionError()

    def calculateTotalPositionError(self):
        """
        This method calculates total position sensor error form errors for coordinates. Output is in cm
        """
        try:
            self.totalPositionError = sqrt(self.positionError[0] ** 2 + self.positionError[1] ** 2
                                       + self.positionError[2] ** 2)
        except:
            self.totalPositionError = None
