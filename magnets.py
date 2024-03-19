#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Aug 03 16:52:07 2022
@author: krzysztof_skos
"""
import math
from random import uniform
import numpy as np


# Given a line with coordinates 'start' and 'end' and the
# coordinates of a point 'pnt' the proc returns the shortest
# distance from pnt to the line and the coordinates of the
# nearest point on the line.
#
# 1  Convert the line segment to a vector ('line_vec').
# 2  Create a vector connecting start to pnt ('pnt_vec').
# 3  Find the length of the line vector ('line_len').
# 4  Convert line_vec to a unit vector ('line_unitvec').
# 5  Scale pnt_vec by line_len ('pnt_vec_scaled').
# 6  Get the dot product of line_unitvec and pnt_vec_scaled ('t').
# 7  Ensure t is in the range 0 to 1.
# 8  Use t to get the nearest location on the line to the end
#    of vector pnt_vec_scaled ('nearest').
# 9  Calculate the distance from nearest to pnt_vec_scaled.
# 10 Translate the nearest back to the start/end line.
# Malcolm Kesson 16 Dec 2012
def dot(v, w):
    """
    This method calculates a dot product of two 3d vectors
    :param v: Vector 1
    :param w: Vector 2
    :return: Dot product of two vectors
    """
    x, y, z = v
    X, Y, Z = w
    return x * X + y * Y + z * Z


def length(v):
    """
    This method calculates a length of 3d vector
    :param v: 3d vector
    :return: Vector length
    """
    x, y, z = v
    return math.sqrt(x * x + y * y + z * z)


def vector(b, e):
    """
    This method calculates the difference of two 3d vectors (e - b)
    :param b: Subtrahend vector
    :param e: Minuend vector
    :return: Vector difference
    """
    x, y, z = b
    X, Y, Z = e
    return X - x, Y - y, Z - z


def unit(v):
    """
    This method converts 3d vector to a unit vector
    :param v: 3d vector
    :return: 3d unit vector
    """
    x, y, z = v
    mag = length(v)
    return x / mag, y / mag, z / mag


def distance(p0, p1):
    """
    This method calculates distance between two points in 3d Cartesian coordinate system
    :param p0: Point 1
    :param p1: Point 2
    :return: Distance between 2 points
    """
    return length(vector(p0, p1))


def scale(v, sc):
    """
    This method scales vector v with parameter sc
    :param v: 3d vector
    :param sc: scaling parameter
    :return: Scaled 3d vector
    """
    x, y, z = v
    return x * sc, y * sc, z * sc


def add(v, w):
    """
    This method adds two 3d vectors
    :param v: Vector 1
    :param w: Vector 2
    :return: Vector sum
    """
    x, y, z = v
    X, Y, Z = w
    return x + X, y + Y, z + Z


def pnt2line(pnt, start, end):
    """
    This method calculates the distance from the point to a segment in 3d Cartesian coordinate system and
    coordinates of the point on the segment nearest to the given point
    :param pnt: Point
    :param start: Segment starting point
    :param end: Segment ending point
    :return: List containing distance and coordinates of the nearest point on the segment
    """
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0 / line_len)
    t = dot(line_unitvec, pnt_vec_scaled)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return dist, nearest

def addErrorToGeomagneticFluxVector(geo_mag):
    # Adding WMM random flux from error distribution
    component_x = 131 / np.power(10, 9)  # Error X component in Tesla (10^(-9) to change unit from nT to T
    component_y = 94 / np.power(10, 9)  # Error Y component in Tesla (10^(-9) to change unit from nT to T
    component_z = 157 / np.power(10, 9)  # Error Z component in Tesla (10^(-9) to change unit from nT to T

    geo_mag[0] *= uniform(-component_x, component_x)
    geo_mag[1] *= uniform(-component_y, component_y)
    geo_mag[2] *= uniform(-component_z, component_z)
    # geo_mag[0] *= component_x
    # geo_mag[1] *= component_y
    # geo_mag[2] *= component_z

    # component_x = 65  / np.power(10, 3)  # Error X component in Tesla (10^(-6) to change unit from uT to T
    # component_y = 65  / np.power(10, 3)  # Error Y component in Tesla (10^(-6) to change unit from uT to T
    # component_z = 65  / np.power(10, 3)  # Error Z component in Tesla (10^(-6) to change unit from uT to T
    #
    # geo_mag[0] *= component_x  # uniform(component_x, component_x+0.00004)
    # # geo_mag[0] *= choice([-1.0, 1.0])
    # geo_mag[1] *= component_y  # uniform(component_y, component_y+0.00004)
    # # geo_mag[1] *= choice([-1.0, 1.0])
    # geo_mag[2] *= component_z  # uniform(component_z, component_z+0.00004)
    # # geo_mag[2] *= choice([-1.0, 1.0])

    return geo_mag

class Magnet:
    """
    This class represents a system of three wires acting as magnets in 3d Cartesian coordinate system
    """
    magnetX = ((0.0, 0.0, 0.0), (101.0, 0.0, 0.0))  # points creating vector X (magnet X) in cm
    magnetY = ((0.0, 0.0, 0.0), (0.0, 101.0, 0.0))  # points creating vector Y (magnet Y) in cm
    magnetZ = ((0.0, 0.0, 0.0), (0.0, 0.0, 201.0))  # points creating vector Z (magnet Z) in cm
    # magneticFlux = (4.0, 4.0, 4.0)  # (0.027, 0.027, 0.027)  # Maximal magnetic flux (near the wire) in T
    # magneticFlux = (27.0, 27.0, 27.0)  # (0.027, 0.027, 0.027)  # Maximal magnetic flux (near the wire) in T
    current = (10.0, 10.0, 10.0) #(135000.0, 135000.0, 135000.0)  # (135.0, 135.0, 135.0)  # Current in magnets in A
    # current = (20000.0, 20000.0, 20000.0)  # (135.0, 135.0, 135.0)  # Current in magnets in A
    noise = (-300 * 10 ** (-6), 300 * 10 ** (-6))  # noise range in T
    # noise = (4 * 10 ** (-6), 6 * 10 ** (-6))  # noise range in T

    def __init__(self):
        """
        Class constructor
        """
        # self.magnetX = ((100.0, 0.0, 100.0), (0.0, 0.0, 100.0))
        # self.magnetY = ((50.0, 0.0, 0.0), (50.0, 100.0, 0.0))
        # self.magnetZ = ((0.0, 50.0, 0.0), (0.0, 50.0, 200.0))
        # 1m from magnets
        self.magnetX = ((0.0, 0.0, 0.0), (101.0, 0.0, 0.0))
        self.magnetY = ((0.0, 0.0, 0.0), (0.0, 101.0, 0.0))
        self.magnetZ = ((0.0, 0.0, 0.0), (0.0, 0.0, 201.0))

        # 0.5m from magnets
        # self.magnetX = ((36.0, 0.0, 0.0), (136.0, 0.0, 0.0))
        # self.magnetY = ((0.0, 36.0, 0.0), (0.0, 136.0, 0.0))
        # self.magnetZ = ((0.0, 0.0, 36.0), (0.0, 0.0, 236.0))

        # 1mm from magnets
        # self.magnetX = ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
        # self.magnetY = ((0.0, 0.0, 0.0), (0.0, 100.0, 0.0))
        # self.magnetZ = ((0.0, 0.0, 0.0), (0.0, 0.0, 220.0))

    def distances(self, point):
        """
        This method calculates distances between point and magnets
        :param point: Vector/list of 3 coordinates (x, y, z) from Cartesian system
        :return: Distances between given point and magnets. Values in cm
        """
        distZ = pnt2line(point, self.magnetZ[0], self.magnetZ[1])
        distY = pnt2line(point, self.magnetY[0], self.magnetY[1])
        distX = pnt2line(point, self.magnetX[0], self.magnetX[1])
        return distX, distY, distZ

    def countFlux(self, distances, position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector):
        """
        This method calculates flux at a given distance for each magnet
        :param distances: Distances from magnets. Value in cm
        :return: list ogf calculated flux. Value in T
        """
        # Adding WMM error to geomagnetic field vector
        # print("Geomagnetic vector 1: ", geomagneticVector)
        geomagneticVector = addErrorToGeomagneticFluxVector(geomagneticVector)
        # print("Geomagnetic vector 2: ", geomagneticVector)

        # fluxX = self.current[0] * 2 * 10 ** (-7) / distances[0] * 100  # *100 to change unit from cm to m
        # print("========================")
        # print("Flux X:")
        # print(fluxX)
        # fluxX = self.addDependentNoise(fluxX)
        fluxX = self.vectorToScalarFluxX(distances[0], position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector)

        # print(fluxX)
        # fluxY = self.current[1] * 2 * 10 ** (-7) / distances[1] * 100  # *100 to change unit from cm to m
        # print("--------------------")
        # print("Flux Y:")
        # print(fluxY)
        # fluxY = self.addDependentNoise(fluxY)
        fluxY = self.vectorToScalarFluxY(distances[1], position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector)

        # print(fluxY)
        # fluxZ = self.current[2] * 2 * 10 ** (-7) / distances[2] * 100  # *100 to change unit from cm to m
        # print("--------------------")
        # print("Flux Z:")
        # print(fluxZ)
        # fluxZ = self.addDependentNoise(fluxZ)
        # print(fluxZ)
        fluxZ = self.vectorToScalarFluxZ(distances[2], position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector)
        # print("========================")
        return fluxX, fluxY, fluxZ

    def vectorToScalarFluxX(self, dist, position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector):
        # Creating generated magnetic flux vector with noise
        B = self.current[0] * 2 / np.power(10, 7) / dist * 100
        # print("B = ", B)
        magnetToSensorVector = np.array([0,
                                         position[1] - self.magnetX[0][1],
                                         position[2] - self.magnetX[0][2]])
        magnetVector = np.array([self.magnetX[1][0] - self.magnetX[0][0],
                                 self.magnetX[1][1] - self.magnetX[0][1],
                                 self.magnetX[1][2] - self.magnetX[0][2]])
        magneticFieldVector = np.cross(magnetVector, magnetToSensorVector)
        magneticFieldVector /= np.linalg.norm(magneticFieldVector)
        magneticFieldVector *= self.addDependentNoise(B)
        # print("Geomagnetic vector: ", geomagneticVector)


        # Generating projection of generated magnetic field to sensor position/rotation vector
        projectionGeneratedX = np.dot(magneticFieldVector, sensorMagX) / math.sqrt(
            np.dot(sensorMagX, sensorMagX)) * sensorMagX
        projectionGeneratedY = np.dot(magneticFieldVector, sensorMagY) / math.sqrt(
            np.dot(sensorMagY, sensorMagY)) * sensorMagY
        projectionGeneratedZ = np.dot(magneticFieldVector, sensorMagZ) / math.sqrt(
            np.dot(sensorMagZ, sensorMagZ)) * sensorMagZ

        # projection magnitude
        projectionGeneratedMagnitudeX = math.sqrt(np.dot(projectionGeneratedX, projectionGeneratedX))
        projectionGeneratedMagnitudeY = math.sqrt(np.dot(projectionGeneratedY, projectionGeneratedY))
        projectionGeneratedMagnitudeZ = math.sqrt(np.dot(projectionGeneratedZ, projectionGeneratedZ))
        # print("Generated magnitude X:", projectionGeneratedMagnitudeX)

        # Generating projection of geomagnetic field to sensor position/rotation vector
        projectionGeomagneticX = np.dot(geomagneticVector, sensorMagX) / math.sqrt(
            np.dot(sensorMagX, sensorMagX)) * sensorMagX
        projectionGeomagneticY = np.dot(geomagneticVector, sensorMagY) / math.sqrt(
            np.dot(sensorMagY, sensorMagY)) * sensorMagY
        projectionGeomagneticZ = np.dot(geomagneticVector, sensorMagZ) / math.sqrt(
            np.dot(sensorMagZ, sensorMagZ)) * sensorMagZ

        # projection magnitude
        projectionGeomagneticMagnitudeX = math.sqrt(np.dot(projectionGeomagneticX, projectionGeomagneticX))
        projectionGeomagneticMagnitudeY = math.sqrt(np.dot(projectionGeomagneticY, projectionGeomagneticY))
        projectionGeomagneticMagnitudeZ = math.sqrt(np.dot(projectionGeomagneticZ, projectionGeomagneticZ))
        # print("Geomagnetic Vector: ", geomagneticVector[0])
        # print("Geo magnitude X:", projectionGeomagneticMagnitudeX)

        # Sum of magnitudes
        magnitudeX = projectionGeneratedMagnitudeX + projectionGeomagneticMagnitudeX
        magnitudeY = projectionGeneratedMagnitudeY + projectionGeomagneticMagnitudeY
        magnitudeZ = projectionGeneratedMagnitudeZ + projectionGeomagneticMagnitudeZ

        # total magnitude
        totalMagnitude = np.sqrt(np.power(magnitudeX, 2) + np.power(magnitudeY, 2) + np.power(magnitudeZ, 2))

        return totalMagnitude

    def vectorToScalarFluxY(self, dist, position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector):
        # Creating generated magnetic flux vector with noise
        B = self.current[0] * 2 / np.power(10, 7) / dist * 100
        magnetToSensorVector = np.array([position[0] - self.magnetY[0][0],
                                         0,
                                         position[2] - self.magnetY[0][2]])
        magnetVector = np.array([self.magnetY[1][0] - self.magnetY[0][0],
                                 self.magnetY[1][1] - self.magnetY[0][1],
                                 self.magnetY[1][2] - self.magnetY[0][2]])
        magneticFieldVector = np.cross(magnetVector, magnetToSensorVector)
        magneticFieldVector /= np.linalg.norm(magneticFieldVector)
        magneticFieldVector *= self.addDependentNoise(B)

        # Generating projection of generated magnetic field to sensor position/rotation vector
        projectionGeneratedX = np.dot(magneticFieldVector, sensorMagX) / math.sqrt(
            np.dot(sensorMagX, sensorMagX)) * sensorMagX
        projectionGeneratedY = np.dot(magneticFieldVector, sensorMagY) / math.sqrt(
            np.dot(sensorMagY, sensorMagY)) * sensorMagY
        projectionGeneratedZ = np.dot(magneticFieldVector, sensorMagZ) / math.sqrt(
            np.dot(sensorMagZ, sensorMagZ)) * sensorMagZ

        # projection magnitude
        projectionGeneratedMagnitudeX = math.sqrt(np.dot(projectionGeneratedX, projectionGeneratedX))
        projectionGeneratedMagnitudeY = math.sqrt(np.dot(projectionGeneratedY, projectionGeneratedY))
        projectionGeneratedMagnitudeZ = math.sqrt(np.dot(projectionGeneratedZ, projectionGeneratedZ))

        # Generating projection of geomagnetic field to sensor position/rotation vector
        projectionGeomagneticX = np.dot(geomagneticVector, sensorMagX) / math.sqrt(
            np.dot(sensorMagX, sensorMagX)) * sensorMagX
        projectionGeomagneticY = np.dot(geomagneticVector, sensorMagY) / math.sqrt(
            np.dot(sensorMagY, sensorMagY)) * sensorMagY
        projectionGeomagneticZ = np.dot(geomagneticVector, sensorMagZ) / math.sqrt(
            np.dot(sensorMagZ, sensorMagZ)) * sensorMagZ

        # projection magnitude
        projectionGeomagneticMagnitudeX = math.sqrt(np.dot(projectionGeomagneticX, projectionGeomagneticX))
        projectionGeomagneticMagnitudeY = math.sqrt(np.dot(projectionGeomagneticY, projectionGeomagneticY))
        projectionGeomagneticMagnitudeZ = math.sqrt(np.dot(projectionGeomagneticZ, projectionGeomagneticZ))

        # Sum of magnitudes
        magnitudeX = projectionGeneratedMagnitudeX + projectionGeomagneticMagnitudeX
        magnitudeY = projectionGeneratedMagnitudeY + projectionGeomagneticMagnitudeY
        magnitudeZ = projectionGeneratedMagnitudeZ + projectionGeomagneticMagnitudeZ

        # total magnitude
        totalMagnitude = np.sqrt(np.power(magnitudeX, 2) + np.power(magnitudeY, 2) + np.power(magnitudeZ, 2))

        return totalMagnitude

    def vectorToScalarFluxZ(self, dist, position, sensorMagX, sensorMagY, sensorMagZ, geomagneticVector):
        # Creating generated magnetic flux vector with noise
        B = self.current[0] * 2 / np.power(10, 7) / dist * 100
        magnetToSensorVector = np.array([position[0] - self.magnetZ[0][0],
                                         position[1] - self.magnetZ[0][1],
                                         0])
        magnetVector = np.array([self.magnetZ[1][0] - self.magnetZ[0][0],
                                 self.magnetZ[1][1] - self.magnetZ[0][1],
                                 self.magnetZ[1][2] - self.magnetZ[0][2]])
        magneticFieldVector = np.cross(magnetVector, magnetToSensorVector)
        magneticFieldVector /= np.linalg.norm(magneticFieldVector)
        magneticFieldVector *= self.addDependentNoise(B)

        # Generating projection of generated magnetic field to sensor position/rotation vector
        projectionGeneratedX = np.dot(magneticFieldVector, sensorMagX) / math.sqrt(
            np.dot(sensorMagX, sensorMagX)) * sensorMagX
        projectionGeneratedY = np.dot(magneticFieldVector, sensorMagY) / math.sqrt(
            np.dot(sensorMagY, sensorMagY)) * sensorMagY
        projectionGeneratedZ = np.dot(magneticFieldVector, sensorMagZ) / math.sqrt(
            np.dot(sensorMagZ, sensorMagZ)) * sensorMagZ

        # projection magnitude
        projectionGeneratedMagnitudeX = math.sqrt(np.dot(projectionGeneratedX, projectionGeneratedX))
        projectionGeneratedMagnitudeY = math.sqrt(np.dot(projectionGeneratedY, projectionGeneratedY))
        projectionGeneratedMagnitudeZ = math.sqrt(np.dot(projectionGeneratedZ, projectionGeneratedZ))

        # Generating projection of geomagnetic field to sensor position/rotation vector
        projectionGeomagneticX = np.dot(geomagneticVector, sensorMagX) / math.sqrt(
            np.dot(sensorMagX, sensorMagX)) * sensorMagX
        projectionGeomagneticY = np.dot(geomagneticVector, sensorMagY) / math.sqrt(
            np.dot(sensorMagY, sensorMagY)) * sensorMagY
        projectionGeomagneticZ = np.dot(geomagneticVector, sensorMagZ) / math.sqrt(
            np.dot(sensorMagZ, sensorMagZ)) * sensorMagZ

        # projection magnitude
        projectionGeomagneticMagnitudeX = math.sqrt(np.dot(projectionGeomagneticX, projectionGeomagneticX))
        projectionGeomagneticMagnitudeY = math.sqrt(np.dot(projectionGeomagneticY, projectionGeomagneticY))
        projectionGeomagneticMagnitudeZ = math.sqrt(np.dot(projectionGeomagneticZ, projectionGeomagneticZ))

        # Sum of magnitudes
        magnitudeX = projectionGeneratedMagnitudeX + projectionGeomagneticMagnitudeX
        magnitudeY = projectionGeneratedMagnitudeY + projectionGeomagneticMagnitudeY
        magnitudeZ = projectionGeneratedMagnitudeZ + projectionGeomagneticMagnitudeZ

        # total magnitude
        totalMagnitude = np.sqrt(np.power(magnitudeX, 2) + np.power(magnitudeY, 2) + np.power(magnitudeZ, 2))

        return totalMagnitude

    def addNoise(self, flux):
        """
        This method adds noise to given flux. Noise is generated from uniform distribution with values:
        min = self.noise[0]
        max = self.noise[1]
        :param flux: Value in T
        :return: flux with noise. Value in T
        """
        flux += uniform(self.noise[0], self.noise[1])
        return flux

    def addDependentNoise(self, flux):
        flux += uniform(-0.01 * flux, 0.01 * flux)
        return flux