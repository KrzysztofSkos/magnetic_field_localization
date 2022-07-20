import math
from random import uniform


def dot(v, w):
    x, y, z = v
    X, Y, Z = w
    return x * X + y * Y + z * Z


def length(v):
    x, y, z = v
    return math.sqrt(x * x + y * y + z * z)


def vector(b, e):
    x, y, z = b
    X, Y, Z = e
    return X - x, Y - y, Z - z


def unit(v):
    x, y, z = v
    mag = length(v)
    return x / mag, y / mag, z / mag


def distance(p0, p1):
    return length(vector(p0, p1))


def scale(v, sc):
    x, y, z = v
    return x * sc, y * sc, z * sc


def add(v, w):
    x, y, z = v
    X, Y, Z = w
    return x + X, y + Y, z + Z


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
# 10 Translate nearest back to the start/end line.
# Malcolm Kesson 16 Dec 2012

def pnt2line(pnt, start, end):
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
    magnetX = ((71.0, 0.0, 0.0), (171.0, 0.0, 0.0))  # points creating vector X (magnet X) in cm
    magnetY = ((0.0, 71.0, 0.0), (0.0, 171.0, 0.0))  # points creating vector Y (magnet Y) in cm
    magnetZ = ((0.0, 0.0, 71.0), (0.0, 0.0, 271.0))  # points creating vector Z (magnet Z) in cm
    magneticFlux = (27.0, 27.0, 27.0)  # (0.027, 0.027, 0.027)  # Maximal magnetic flux (near the wire) in T
    current = (135000.0, 135000.0, 135000.0)  # (135.0, 135.0, 135.0)  # Current in magnets in A
    noise = (80 * 10 ** (-6), 300 * 10 ** (-6))  # noise range in T

    def __init__(self):
        # self.magnetX = ((100.0, 0.0, 100.0), (0.0, 0.0, 100.0))
        # self.magnetY = ((50.0, 0.0, 0.0), (50.0, 100.0, 0.0))
        # self.magnetZ = ((0.0, 50.0, 0.0), (0.0, 50.0, 200.0))
        self.magnetX = ((71.0, 0.0, 0.0), (171.0, 0.0, 0.0))
        self.magnetY = ((0.0, 71.0, 0.0), (0.0, 171.0, 0.0))
        self.magnetZ = ((0.0, 0.0, 71.0), (0.0, 0.0, 271.0))

    def distances(self, point):  # in cm
        distZ = pnt2line(point, self.magnetZ[0], self.magnetZ[1])
        distY = pnt2line(point, self.magnetY[0], self.magnetY[1])
        distX = pnt2line(point, self.magnetX[0], self.magnetX[1])
        return distX, distY, distZ

    def countFlux(self, distances):
        fluxX = self.current[0] * 2 * 10 ** (-7) / distances[0] * 100  # *100 to change unit from cm to m
        # print("========================")
        # print("Flux X:")
        # print(fluxX)
        fluxX = self.addNoise(fluxX)
        # print(fluxX)
        fluxY = self.current[1] * 2 * 10 ** (-7) / distances[1] * 100  # *100 to change unit from cm to m
        # print("--------------------")
        # print("Flux Y:")
        # print(fluxY)
        fluxY = self.addNoise(fluxY)
        # print(fluxY)
        fluxZ = self.current[2] * 2 * 10 ** (-7) / distances[2] * 100  # *100 to change unit from cm to m
        # print("--------------------")
        # print("Flux Z:")
        # print(fluxZ)
        fluxZ = self.addNoise(fluxZ)
        # print(fluxZ)
        # print("========================")
        return fluxX, fluxY, fluxZ

    def addNoise(self, flux):
        flux += uniform(self.noise[0], self.noise[1])
        return flux
