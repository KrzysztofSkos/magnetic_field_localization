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
    distanceX = []
    distanceY = []
    distanceZ = []
    fluxX = []
    fluxY = []
    fluxZ = []
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


    def setDistance15(self, distancesX, distancesY, distancesZ):
        self.distanceX = []
        self.distanceY = []
        self.distanceZ = []
        for dist in distancesX:
            self.distanceX.append(dist)

        for dist in distancesY:
            self.distanceY.append(dist)

        for dist in distancesZ:
            self.distanceZ.append(dist)


    def setFlux15(self, fluxesX, fluxesY, fluxesZ):
        self.fluxX = []
        self.fluxY = []
        self.fluxZ = []

        for flux in fluxesX:
            self.fluxX.append(flux)

        for flux in fluxesY:
            self.fluxY.append(flux)

        for flux in fluxesZ:
            self.fluxZ.append(flux)


    def calculateEstimatedDistance15(self, current):

        for i in range(len(self.fluxX)):
            self.distanceEstimatedX[i] = current[0] * 2 / np.power(10, 7) / self.fluxX[i] * 100
        for i in range(len(self.fluxY)):
            self.distanceEstimatedY[i] = current[1] * 2 / np.power(10, 7) / self.fluxY[i] * 100
        for i in range(len(self.fluxZ)):
            self.distanceEstimatedZ[i] = current[2] * 2 / np.power(10, 7) / self.fluxZ[i] * 100


    def calculateEstimatedPosition15_X(self):
        y1 = 242.0 #100.0
        z1 = 171.0 #100.0
        z2 = 342.0 #200.0
        A = [[y1, 0.0], # y1, 0
             [0.0, z1], # 0, z1
             [0.0, z2], # 0, z2
             [y1, z2]] # y1, z2
        B = [(np.power(self.distanceEstimatedX[0], 2) - np.power(self.distanceEstimatedX[1], 2) + np.power(y1, 2)) / 2,
             (np.power(self.distanceEstimatedX[0], 2) - np.power(self.distanceEstimatedX[2], 2) + np.power(z1, 2)) / 2,
             (np.power(self.distanceEstimatedX[0], 2) - np.power(self.distanceEstimatedX[3], 2) + np.power(z2, 2)) / 2,
             (np.power(self.distanceEstimatedX[0], 2) - np.power(self.distanceEstimatedX[4], 2) + np.power(y1, 2) + np.power(z2, 2)) / 2]
        A = np.array(A)
        B = np.array(B)
        # R = (np.transpose(A) * A) ** (-1) * np.transpose(A) * B
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
        B = [(np.power(self.distanceEstimatedY[0], 2) - np.power(self.distanceEstimatedY[1], 2) + np.power(x1, 2)) / 2,
             (np.power(self.distanceEstimatedY[0], 2) - np.power(self.distanceEstimatedY[2], 2) + np.power(z1, 2)) / 2,
             (np.power(self.distanceEstimatedY[0], 2) - np.power(self.distanceEstimatedY[3], 2) + np.power(x1, 2) + np.power(z1, 2)) / 2,
             (np.power(self.distanceEstimatedY[0], 2) - np.power(self.distanceEstimatedY[4], 2) + np.power(z2, 2)) / 2,
             (np.power(self.distanceEstimatedY[0], 2) - np.power(self.distanceEstimatedY[5], 2) + np.power(x1, 2) + np.power(z2, 2)) / 2]
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
        B = [(np.power(self.distanceEstimatedZ[0], 2) - np.power(self.distanceEstimatedZ[1], 2) + np.power(x1, 2)) / 2,
             (np.power(self.distanceEstimatedZ[0], 2) - np.power(self.distanceEstimatedZ[2], 2) + np.power(y1, 2)) / 2,
             (np.power(self.distanceEstimatedZ[0], 2) - np.power(self.distanceEstimatedZ[3], 2) + np.power(x1, 2) + np.power(y1, 2)) / 2]
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
            self.totalPositionError = sqrt(np.power(self.positionError[0], 2) + np.power(self.positionError[1], 2)
                                       + np.power(self.positionError[2], 2))
        except:
            self.totalPositionError = None
