#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Aug 03 16:52:07 2022
@author: krzysztof_skos
"""
from math import sqrt, sin, cos, pi
import random
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
    flux = [0.0, 0.0, 0.0]
    positionEstimated = [0.0, 0.0, 0.0]
    distanceEstimated = [0.0, 0.0, 0.0]
    positionError = [0.0, 0.0, 0.0]
    positionErrorX = 0.0
    positionErrorY = 0.0
    positionErrorZ = 0.0
    totalPositionError = 0.0
    sensorMagX = []
    sensorMagY = []
    sensorMagZ = []

    def __init__(self, pos):
        """
        Sensor constructor
        :param pos: Sensor position in 3d Cartesian coordinate system. Value in cm
        """
        self.position = pos
        # self.setGenerateSensorRotation()

    def setGenerateSensorRotation(self):
        # Generating first vector
        sensorTheta = random.uniform(0, 2 * pi)
        sensorZ = random.uniform(-1, 1)
        sensorMagX = np.array(
            [sqrt(1 - np.power(sensorZ, 2)) * sin(sensorTheta), sqrt(1 - np.power(sensorZ, 2)) * cos(sensorTheta),
             sensorZ])
        # Generating second vector
        sensorMagY = np.random.randn(3)
        sensorMagY -= sensorMagY.dot(sensorMagX) * sensorMagX
        sensorMagY /= np.linalg.norm(sensorMagY)
        # Generating third vector
        sensorMagZ = np.cross(sensorMagX, sensorMagY)
        # Set values
        self.sensorMagX = sensorMagX
        self.sensorMagY = sensorMagY
        self.sensorMagZ = sensorMagZ

    def setDistance(self, dist):
        """
        Distance setter.
        :param dist: Distance between magnet and sensor. Value in ?
        """
        self.distance[0] = dist[0]
        self.distance[1] = dist[1]
        self.distance[2] = dist[2]

    def setFlux(self, flux):
        """
        Flux setter.
        :param flux: flux measured by sensor. Value in T
        """
        self.flux[0] = flux[0]
        self.flux[1] = flux[1]
        self.flux[2] = flux[2]

    def calculateEstimatedDistance(self, current):
        """
        This method estimates distance from measured flux. Output is in cm
        """
        self.distanceEstimated[0] = current[0] * 2 * 10 ** (-7) / self.flux[0] * 100  # *100 to change unit from cm to m
        self.distanceEstimated[1] = current[1] * 2 * 10 ** (-7) / self.flux[1] * 100  # *100 to change unit from cm to m
        self.distanceEstimated[2] = current[2] * 2 * 10 ** (-7) / self.flux[2] * 100  # *100 to change unit from cm to m

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
        self.positionErrorX = self.positionError[0]
        self.positionErrorY = self.positionError[1]
        self.positionErrorZ = self.positionError[2]
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
