class Sensor:
    position = (0.0, 0.0, 0.0)  # Position (x, y, z)
    distance = (0.0, 0.0, 0.0)
    flux = (0.0, 0.0, 0.0)

    def __init__(self, pos):
        self.position = pos

    def setDistance(self, dist):
        self.distance = dist

    def setFlux(self, flux):
        self.flux = flux
