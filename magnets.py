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


class Magnet:
    """
    This class represents a system of three wires acting as magnets in 3d Cartesian coordinate system
    """
    # magnetX1 = ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))  # points creating vector X (magnet X) in cm
    # magnetX2 = ((0.0, 100.0, 0.0), (100.0, 100.0, 0.0))
    # magnetX3 = ((0.0, 0.0, 100.0), (100.0, 0.0, 100.0))
    # magnetX4 = ((0.0, 0.0, 200.0), (100.0, 0.0, 200.0))
    # magnetX5 = ((0.0, 100.0, 200.0), (100.0, 100.0, 200.0))
    #
    # magnetY1 = ((0.0, 0.0, 0.0), (0.0, 100.0, 0.0))  # points creating vector Y (magnet Y) in cm
    # magnetY2 = ((100.0, 0.0, 0.0), (100.0, 100.0, 0.0))
    # magnetY3 = ((0.0, 0.0, 100.0), (0.0, 100.0, 100.0))
    # magnetY4 = ((100.0, 0.0, 100.0), (100.0, 100.0, 100.0))
    # magnetY5 = ((0.0, 0.0, 200.0), (0.0, 100.0, 200.0))
    # magnetY6 = ((100.0, 0.0, 200.0), (100.0, 100.0, 200.0))
    #
    # magnetZ1 = ((0.0, 0.0, 0.0), (0.0, 0.0, 200.0))  # points creating vector Z (magnet Z) in cm
    # magnetZ2 = ((100.0, 100.0, 0.0), (100.0, 100.0, 200.0))
    # magnetZ3 = ((0.0, 100.0, 0.0), (0.0, 100.0, 200.0))
    # magnetZ4 = ((100.0, 0.0, 0.0), (100.0, 0.0, 200.0))

    # magnetX1 = ((71.0, 0.0, 0.0), (171.0, 0.0, 0.0))  # points creating vector X (magnet X) in cm
    # magnetX2 = ((71.0, 242.0, 0.0), (171.0, 242.0, 0.0))
    # magnetX3 = ((71.0, 0.0, 171.0), (171.0, 0.0, 171.0))
    # magnetX4 = ((71.0, 0.0, 342.0), (171.0, 0.0, 342.0))
    # magnetX5 = ((71.0, 242.0, 342.0), (171.0, 242.0, 342.0))
    #
    # magnetY1 = ((0.0, 71.0, 0.0), (0.0, 171.0, 0.0))  # points creating vector Y (magnet Y) in cm
    # magnetY2 = ((242.0, 71.0, 0.0), (242.0, 171.0, 0.0))
    # magnetY3 = ((0.0, 71.0, 171.0), (0.0, 171.0, 171.0))
    # magnetY4 = ((242.0, 71.0, 171.0), (242.0, 171.0, 171.0))
    # magnetY5 = ((0.0, 71.0, 342.0), (0.0, 171.0, 342.0))
    # magnetY6 = ((242.0, 71.0, 342.0), (242.0, 171.0, 342.0))
    #
    # magnetZ1 = ((0.0, 0.0, 71.0), (0.0, 0.0, 271.0))  # points creating vector Z (magnet Z) in cm
    # magnetZ2 = ((242.0, 242.0, 71.0), (242.0, 242.0, 271.0))
    # magnetZ3 = ((0.0, 242.0, 71.0), (0.0, 242.0, 271.0))
    # magnetZ4 = ((242.0, 0.0, 71.0), (242.0, 0.0, 271.0))
    magnetsX = []
    magnetsY = []
    magnetsZ = []
    # magnetX1 = ((0.0, 0.0, 0.0), (242.0, 0.0, 0.0))  # points creating vector X (magnet X) in cm
    # magnetX2 = ((0.0, 242.0, 0.0), (242.0, 242.0, 0.0))
    # magnetX3 = ((0.0, 0.0, 171.0), (242.0, 0.0, 171.0))
    # magnetX4 = ((0.0, 0.0, 342.0), (242.0, 0.0, 342.0))
    # magnetX5 = ((0.0, 242.0, 342.0), (242.0, 242.0, 342.0))

    # magnetY1 = ((0.0, 0.0, 0.0), (0.0, 242.0, 0.0))  # points creating vector Y (magnet Y) in cm
    # magnetY2 = ((242.0, 0.0, 0.0), (242.0, 242.0, 0.0))
    # magnetY3 = ((0.0, 0.0, 171.0), (0.0, 242.0, 171.0))
    # magnetY4 = ((242.0, 0.0, 171.0), (242.0, 242.0, 171.0))
    # magnetY5 = ((0.0, 0.0, 342.0), (0.0, 242.0, 342.0))
    # magnetY6 = ((242.0, 0.0, 342.0), (242.0, 242.0, 342.0))
    #
    # magnetZ1 = ((0.0, 0.0, 0.0), (0.0, 0.0, 342.0))  # points creating vector Z (magnet Z) in cm
    # magnetZ2 = ((242.0, 0.0, 0.0), (242.0, 0.0, 342.0))
    # magnetZ3 = ((0.0, 242.0, 0.0), (0.0, 242.0, 342.0))
    # magnetZ4 = ((242.0, 242.0, 0.0), (242.0, 242.0, 342.0))

    # magneticFlux = (4.0, 4.0, 4.0)  # (0.027, 0.027, 0.027)  # Maximal magnetic flux (near the wire) in T
    magneticFlux = (27.0, 27.0, 27.0)  # (0.027, 0.027, 0.027)  # (27.0, 27.0, 27.0)   # Maximal magnetic flux (near the wire) in T
    current = (135000.0, 135000.0, 135000.0) # (135000.0, 135000.0, 135000.0)  # (135.0, 135.0, 135.0)  # Current in magnets in A [Amperes]
    # current = (20000.0, 20000.0, 20000.0)  # (135.0, 135.0, 135.0)  # Current in magnets in A
    noise = (-300 / np.power(10, 6), 300 / np.power(10, 6))  # noise range in T
    # noise = (-5 * np.power(10, -6), 5 * np.power(10, -6))
    # noise = (4 * np.power(10, -6), 6 * np.power(10, -6))  # noise range in T

    def __init__(self):
        """
        Class constructor
        """
        # points creating vector X (magnet X) in cm
        self.magnetsX.append(((0.0, 0.0, 0.0), (242.0, 0.0, 0.0)))
        self.magnetsX.append(((0.0, 242.0, 0.0), (242.0, 242.0, 0.0)))
        self.magnetsX.append(((0.0, 0.0, 171.0), (242.0, 0.0, 171.0)))
        self.magnetsX.append(((0.0, 0.0, 342.0), (242.0, 0.0, 342.0)))
        self.magnetsX.append(((0.0, 242.0, 342.0), (242.0, 242.0, 342.0)))
        # points creating vector Y (magnet Y) in cm
        self.magnetsY.append(((0.0, 0.0, 0.0), (0.0, 242.0, 0.0)))
        self.magnetsY.append(((242.0, 0.0, 0.0), (242.0, 242.0, 0.0)))
        self.magnetsY.append(((0.0, 0.0, 171.0), (0.0, 242.0, 171.0)))
        self.magnetsY.append(((242.0, 0.0, 171.0), (242.0, 242.0, 171.0)))
        self.magnetsY.append(((0.0, 0.0, 342.0), (0.0, 242.0, 342.0)))
        self.magnetsY.append(((242.0, 0.0, 342.0), (242.0, 242.0, 342.0)))
        # points creating vector Z (magnet Z) in cm
        self.magnetsZ.append(((0.0, 0.0, 0.0), (0.0, 0.0, 342.0)))
        self.magnetsZ.append(((242.0, 0.0, 0.0), (242.0, 0.0, 342.0)))
        self.magnetsZ.append(((0.0, 242.0, 0.0), (0.0, 242.0, 342.0)))
        self.magnetsZ.append(((242.0, 242.0, 0.0), (242.0, 242.0, 342.0)))
        # self.magnetX = ((100.0, 0.0, 100.0), (0.0, 0.0, 100.0))
        # self.magnetY = ((50.0, 0.0, 0.0), (50.0, 100.0, 0.0))
        # self.magnetZ = ((0.0, 50.0, 0.0), (0.0, 50.0, 200.0))
        # 1m from magnets
        # self.magnetX = ((71.0, 0.0, 0.0), (171.0, 0.0, 0.0))
        # self.magnetY = ((0.0, 71.0, 0.0), (0.0, 171.0, 0.0))
        # self.magnetZ = ((0.0, 0.0, 71.0), (0.0, 0.0, 271.0))

        # 0.5m from magnets
        # self.magnetX = ((36.0, 0.0, 0.0), (136.0, 0.0, 0.0))
        # self.magnetY = ((0.0, 36.0, 0.0), (0.0, 136.0, 0.0))
        # self.magnetZ = ((0.0, 0.0, 36.0), (0.0, 0.0, 236.0))

        # 1mm from magnets
        # self.magnetX = ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
        # self.magnetY = ((0.0, 0.0, 0.0), (0.0, 100.0, 0.0))
        # self.magnetZ = ((0.0, 0.0, 0.0), (0.0, 0.0, 220.0))

    def distances15(self, point):
        distX = []
        distY = []
        distZ = []

        for magnet in self.magnetsX:
            distX.append((pnt2line(point, magnet[0], magnet[1])[0]))

        for magnet in self.magnetsY:
            distY.append((pnt2line(point, magnet[0], magnet[1])[0]))

        for magnet in self.magnetsZ:
            distZ.append((pnt2line(point, magnet[0], magnet[1])[0]))

        return distX, distY, distZ

    def countFlux15(self, distancesX, distancesY, distancesZ):
        fluxX = []
        fluxY = []
        fluxZ = []

        for dist in distancesX:
            temp = self.current[0] * 2 / np.power(10, 7) / dist * 100
            fluxX.append(self.addNoise(temp))

        for dist in distancesY:
            temp = self.current[0] * 2 / np.power(10, 7) / dist * 100
            fluxY.append(self.addNoise(temp))

        for dist in distancesZ:
            temp = self.current[0] * 2 / np.power(10, 7) / dist * 100
            fluxZ.append(self.addNoise(temp))

        return fluxX, fluxY, fluxZ


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
