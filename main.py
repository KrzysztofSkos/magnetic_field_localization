#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Aug 03 16:52:07 2022
@author: krzysztof_skos
"""
from numpy import mean
from magnets import Magnet
from sensor import Sensor
import matplotlib.pyplot as plt
import matplotlib.colors
import csv


def meanOfList(list1):
    """
    This method calculates means of first, second and third values of list of three element lists
    :param list1: List of 3 element lists
    :return: list of means
    """
    X = 0
    Y = 0
    Z = 0
    for fl in list1:
        X += fl[0]
        Y += fl[1]
        Z += fl[2]
    lenList = len(list1)
    X = X / lenList
    Y = Y / lenList
    Z = Z / lenList
    return X, Y, Z


magnet = Magnet()

# Generating points
# x -> 71:171
# y -> 71:171
# z -> 71:271
points = []
for x in range(71, 171, 10):
    for y in range(71, 171, 10):
        for z in range(71, 271, 10):
            points.append(Sensor((float(x), float(y), float(z))))
# points.append(Sensor((float(100.0), float(100.0), float(100.0))))

# Counting the flux
meanFluxList = []
for point in points:
    fluxList = []
    errorList = []
    totalErrorList = []
    temp = magnet.distances(point.position)
    point.setDistance((temp[0][0], temp[1][0], temp[2][0]))
    for i in range(0, 10):
        point.setFlux(magnet.countFlux(point.distance))
        fluxList.append(point.flux)
        point.calculateEstimatedDistance(magnet.current)
        point.calculateEstimatedPosition()
        point.calculatePositionError()
        # print("===========================")
        # print(point.flux)
        # print("Position")
        # print(point.position)
        # print(point.positionEstimated)
        # print("Distance")
        # print(point.distance)
        # print(point.distanceEstimated)
        # print("Error")
        # print(point.positionError)
        errorList.append(point.positionError)
        totalErrorList.append(point.totalPositionError)

    fX, fY, fZ = meanOfList(fluxList)
    eX, eY, eZ = meanOfList(errorList)

    meanFluxList.append([point.position, (fX, fY, fZ), (eX, eY, eZ), mean(totalErrorList)])

print(meanFluxList[0])
print(len(meanFluxList))

f = open('test2.csv', 'w')
writer = csv.writer(f)

writer.writerow(("X", "Y", "Z", "Received flux X", "Received flux Y", "Received flux Z",
                 "Error X", "Error Y", "Error Z", "Total position error"))
for row in meanFluxList:
    # print(row)
    writer.writerow((row[0][0], row[0][1], row[0][2], row[1][0], row[1][1],
                     row[1][2], row[2][0], row[2][1], row[2][2], row[3]))
f.close()

# # Counting max error
# maxX = 0
# maxY = 0
# maxZ = 0
# for row in meanFluxList:
#     if row[2][0] > maxX:
#         maxX = row[2][0]
#     if row[2][1] > maxY:
#         maxY = row[2][1]
#     if row[2][2] > maxZ:
#         maxZ = row[2][2]
# print((maxX, maxY, maxZ))
#
# # with plt.ion():
#
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# normalize = matplotlib.colors.Normalize(vmin=0, vmax=maxX)
# # Rysowanie błędu dla magnesu X
# for row in meanFluxList:
#     ax.scatter3D(row[0][0], row[0][1], row[0][2], c=row[2][0], norm=normalize)
# ax.view_init(0, 0)
# fig.show()
#
