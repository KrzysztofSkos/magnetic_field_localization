from magnets import Magnet
from sensor import Sensor
import csv

magnet = Magnet()

# Generating points
# x -> 1:100
# y -> 1:100
# z -> 1:200
points = []
for x in range(1, 100, 1):
    for y in range(1, 100, 1):
        for z in range(1, 200, 1):
            points.append(Sensor((float(x), float(y), float(z))))

print(len(points))

# Counting the flux
meanFluxList = []
for point in points:
    fluxList = []
    temp = magnet.distances(point.position)
    point.setDistance((temp[0][0], temp[1][0], temp[2][0]))
    for i in range(0, 10):
        point.setFlux(magnet.countFlux(point.distance))
        fluxList.append(point.flux)
    fX = 0
    fY = 0
    fZ = 0
    for fl in fluxList:
        fX += fl[0]
        fY += fl[1]
        fZ += fl[2]
    lenFluxList = len(fluxList)
    fX = fX/lenFluxList
    fY = fY/lenFluxList
    fZ = fZ/lenFluxList
    meanFluxList.append([point.position, (fX, fY, fZ)])

print(meanFluxList[0])
print(len(meanFluxList))

f = open('test.csv', 'w')
writer = csv.writer(f)

for row in meanFluxList:
    # print(row)
    writer.writerow((row[0][0], row[0][1], row[0][2], row[1][0], row[1][1],
                     row[1][2]))
f.close()
# TODO save meanFluxList as xlsx or plot3D
