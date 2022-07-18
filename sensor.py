from math import sqrt


class Sensor:
    position = (0.0, 0.0, 0.0)  # Position (x, y, z)
    distance = [0.0, 0.0, 0.0]  # Distance calculated
    flux = [0.0, 0.0, 0.0]
    positionEstimated = [0.0, 0.0, 0.0]
    distanceEstimated = [0.0, 0.0, 0.0]
    positionError = [0.0, 0.0, 0.0]

    def __init__(self, pos):
        # self.position[0] = pos[0]
        # self.position[1] = pos[1]
        # self.position[2] = pos[2]
        self.position = pos
        # print(self.position)

    #
    # def printPosition(self):
    #     print(self.position)

    def setDistance(self, dist):
        self.distance[0] = dist[0]
        self.distance[1] = dist[1]
        self.distance[2] = dist[2]

    def setFlux(self, flux):
        self.flux[0] = flux[0]
        self.flux[1] = flux[1]
        self.flux[2] = flux[2]

    def calculateEstimatedDistance(self, current):
        """
        Estimate distance from flux
        :return:
        """
        self.distanceEstimated[0] = current[0] * 2 * 10 ** (-7) / self.flux[0] * 100  # *100 to change unit from cm to m
        self.distanceEstimated[1] = current[1] * 2 * 10 ** (-7) / self.flux[1] * 100  # *100 to change unit from cm to m
        self.distanceEstimated[2] = current[2] * 2 * 10 ** (-7) / self.flux[2] * 100  # *100 to change unit from cm to m

    def calculateEstimatedPosition(self):
        # TODO fix distance calculation  algorithm
        a = abs(self.distanceEstimated[1] ** 2 + self.distanceEstimated[2] ** 2 -
                self.distanceEstimated[0] ** 2) / 2
        print(a)
        self.positionEstimated[0] = sqrt(a)
        b = abs(self.distanceEstimated[0] ** 2 + self.distanceEstimated[2] ** 2 -
                self.distanceEstimated[1] ** 2) / 2
        print(b)
        self.positionEstimated[1] = sqrt(b)
        c = abs(self.distanceEstimated[0] ** 2 + self.distanceEstimated[1] ** 2 -
                self.distanceEstimated[2] ** 2) / 2
        print(c)
        self.positionEstimated[2] = sqrt(c)


    def calculatePositionError(self):
        self.positionError[0] = abs(self.position[0] - self.positionEstimated[0])
        self.positionError[1] = abs(self.position[1] - self.positionEstimated[1])
        self.positionError[2] = abs(self.position[2] - self.positionEstimated[2])
