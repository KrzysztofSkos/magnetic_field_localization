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
import pandas as pd
import numpy as np
import random
import math

tryCounter = 0
x1 = 101.0
y1 = 101.0
z1 = 101.0
z2 = 201.0

def meanListOfLists(listOfLists):
    """
    This method takes a list of lists and returns the list with means od each elements
    :param listOfLists: A list of lists
    :return: finalList: List of means
    """
    finalList = []
    for i in range(len(listOfLists[0])):
        temp = 0
        for element in listOfLists:
            temp += element[i]
        temp /= len(listOfLists)
        finalList.append(temp)
    return finalList


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

def generateGeomagneticFluxVector():
    geo_theta = random.uniform(0, 2 * math.pi)
    geo_z = random.uniform(-1, 1)
    geo_mag = np.array(
        [math.sqrt(1 - np.power(geo_z, 2)) * math.sin(geo_theta),
         math.sqrt(1 - np.power(geo_z, 2)) * math.cos(geo_theta),
         geo_z])
    return geo_mag



magnet = Magnet(x1, y1, z1, z2)
geomagneticVector = generateGeomagneticFluxVector()

df = pd.read_csv("models/finalbasemesh_57x100x19.csv")
data = df.to_numpy()

lookup2 = []
for row in data:
    lookup2.append({ 'x':    row[0], 'y':    row[1], 'z':     row[2]})
# Voxel scaling
scale = 1.77 # 3.43 # mean height = 175cm / max height in voxel model = 51
for i in range(len(lookup2)):
    lookup2[i]['x'] *= scale
    lookup2[i]['y'] *= scale
    lookup2[i]['z'] *= scale

minX = 1000
minY = 1000
minZ = 1000
maxX = 0
maxY = 0
maxZ = 0
for i in range(len(lookup2)):
  if lookup2[i]['x'] <= minX:
    minX = lookup2[i]['x']
  elif lookup2[i]['x'] >= maxX:
    maxX = lookup2[i]['x']
  if lookup2[i]['y'] <= minY:
    minY = lookup2[i]['y']
  elif lookup2[i]['y'] >= maxY:
    maxY = lookup2[i]['y']
  if lookup2[i]['z'] <= minZ:
    minZ = lookup2[i]['z']
  elif lookup2[i]['z'] >= maxZ:
    maxZ = lookup2[i]['z']

print("min x " + str(minX))
print("max x " + str(maxX))
print("min y " + str(minY))
print("max y " + str(maxY))
print("min z " + str(minZ))
print("max z " + str(maxZ))

points = []
for i in range(len(lookup2)):
    # points.append(Sensor((float(lookup2[i]['x']+71), float(lookup2[i]['z']+71), float(lookup2[i]['y']+71)), x1, y1, z1, z2))
    points.append(Sensor((float(lookup2[i]['x']+1), float(lookup2[i]['z']+31), float(lookup2[i]['y']+1)), x1, y1, z1, z2))
# for x in range(71, 171, 10):
#     for y in range(71, 171, 10):
#         for z in range(71, 271, 10):

# points.append(Sensor((float(100.0), float(100.0), float(100.0))))

# Counting the flux
meanFluxList = []
fluxListMeanX = []

# counter = 0
for point in points:
    # if counter >= 10:
    #     break
    # counter += 1
    fluxListX = []
    fluxListY = []
    fluxListZ = []
    errorList = []
    totalErrorList = []
    tempX, tempY, tempZ = magnet.distances15(point.position)
    # print(temp)
    point.setDistance15(tempX, tempY, tempZ)
    for i in range(0, 100):
        point.setGenerateSensorRotation()
        # print (point.distanceX + point.distanceY + point.distanceZ)
        tempX, tempY, tempZ = magnet.countFlux15(point.distanceX, point.distanceY, point.distanceZ, point.position, point.sensorMagX, point.sensorMagY, point.sensorMagZ, geomagneticVector)
        point.setFlux15(tempX, tempY, tempZ)


        fluxListX.append(point.fluxX) # List of lists of flux X for every wire
        fluxListY.append(point.fluxY)
        fluxListZ.append(point.fluxZ)

        #fluxList.append(point.flux)
        point.calculateEstimatedDistance15(magnet.current)
        point.calculateEstimatedPosition15()
        point.calculatePositionError()
        # errorList.append(point.positionError)
        errorList.append((point.positionErrorX, point.positionErrorY, point.positionErrorZ))
        totalErrorList.append(point.totalPositionError)
    # Means of 100 measurements, for every wire
    fX = meanListOfLists(fluxListX)
    fY = meanListOfLists(fluxListY)
    fZ = meanListOfLists(fluxListZ)

    # fX, fY, fZ = meanOfList(fluxList)
    eX, eY, eZ = meanOfList(errorList)

    # meanFluxList.append([point.position, (fX, fY, fZ), (eX, eY, eZ), meanOfError(totalErrorList)])
    # meanFluxList.append([point.position, (0, 0, 0), (eX, eY, eZ), meanOfError(totalErrorList)])
    meanFluxList.append([point.position, (0, 0, 0), (eX, eY, eZ), meanOfError(totalErrorList), (fX, fY, fZ)])

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

f = open('test3_human_body_15_magnets_Graphene_100_repeats_with_geomagnetic_field_current_100_after_debug_with_flux2.csv', 'w')
writer = csv.writer(f)

writer.writerow(("X", "Y", "Z", "Received flux X (not in use)", "Received flux Y (not in use)", "Received flux Z (not in use)",
                 "Error X", "Error Y", "Error Z", "Total position error",
                 "Flux x1", "Flux x2", "Flux x3", "Flux x4", "Flux x5",
                 "Flux y1", "Flux y2", "Flux y3", "Flux y4", "Flux y5", "Flux y6",
                 "Flux z1", "Flux z2", "Flux z3", "Flux z4"))

minX = 1000
minY = 1000
minZ = 1000
maxX = 0
maxY = 0
maxZ = 0
errX = 0
errY = 0
errZ = 0

for row in meanFluxList:
    # print(row)
    writer.writerow((row[0][0], row[0][1], row[0][2], row[1][0], row[1][1],
                     row[1][2], row[2][0], row[2][1], row[2][2], row[3],
                     row[4][0][0], row[4][0][1], row[4][0][2], row[4][0][3], row[4][0][4], # Flux X
                     row[4][1][0], row[4][1][1], row[4][1][2], row[4][1][3], row[4][1][4], row[4][1][5], # Flux Y
                     row[4][2][0], row[4][2][1], row[4][2][2], row[4][2][3] # Flux Z
                     ))
    if row[0][0] <= minX:
      minX = row[0][0]
    elif row[0][0] >= maxX:
      maxX = row[0][0]
    if row[0][1] <= minY:
      minY = row[0][1]
    elif row[0][1] >= maxY:
      maxY = row[0][1]
    if row[0][2] <= minZ:
      minZ = row[0][2]
    elif row[0][2] >= maxZ:
      maxZ = row[0][2]
    errX += row[2][0]
    errY += row[2][1]
    errZ += row[2][2]


f.close()

# print("min x " + str(minX))
# print("max x " + str(maxX))
# print("min y " + str(minY))
# print("max y " + str(maxY))
# print("min z " + str(minZ))
# print("max z " + str(maxZ))
print("error X " + str(errX/len(meanFluxList)))
print("error Y " + str(errY/len(meanFluxList)))
print("error Z " + str(errZ/len(meanFluxList)))
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
