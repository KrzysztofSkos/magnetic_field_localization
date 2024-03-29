#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Aug 03 16:52:07 2022
@author: krzysztof_skos
"""
from numpy import mean
from magnets import Magnet
from sensor import Sensor
import csv

tryCounter = 0


def meanOfList(list1):
    """
    This method calculates means of first, second and third values of list of three element lists
    :param list1: List of 3 element lists
    :return: list of means
    """
    global tryCounter
    X = 0
    Y = 0
    Z = 0
    counterX = 0
    counterY = 0
    counterZ = 0
    for fl in list1:
        if fl[0] is not None:
            X += fl[0]
            counterX += 1
        else:
            tryCounter += 1
        if fl[1] is not None:
            Y += fl[1]
            counterY += 1
        else:
            tryCounter += 1
        if fl[2] is not None:
            Z += fl[2]
            counterZ += 1
        else:
            tryCounter += 1
    if counterX != 0:
        X = X / counterX
    else:
        X = None
    if counterY != 0:
        Y = Y / counterY
    else:
        Y = None
    if counterZ != 0:
        Z = Z / counterZ
    else:
        Z = None
    return X, Y, Z


def meanOfError(list1):
    """
        This method calculates means of a list that may contain None values
        :param list1: List of values
        :return: Mean of not None values
        """
    X = 0
    counterX = 0
    for fl in list1:
        if fl is not None:
            X += fl
            counterX += 1
    if counterX != 0:
        X = X / counterX
    else:
        print("uwaga")
        X = None
    return X


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

    meanFluxList.append([point.position, (fX, fY, fZ), (eX, eY, eZ), meanOfError(totalErrorList)])

print(meanFluxList[0])
print(len(meanFluxList))

lista = []
for la in meanFluxList:
    lista.append(la[3])
print(lista)
con = 0
lista2 = []
for la in lista:
    if la is not None:
        lista2.append(la)
    else:
        con += 1
print(con)
print(mean(lista2))
print(tryCounter)

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
