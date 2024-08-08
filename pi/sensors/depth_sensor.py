import pi.ms5837 as ms5837


# wrapper class to get data out of the depth sensor
class DepthSensor:
    def __init__(self, bus = 1, density = ms5837.DENSITY_FRESHWATER):
        self.sensor = ms5837.MS5837_30BA(bus)
        # default I2C bus is 1 (pi 3)

        if not self.sensor.init():
            print("Sensor could not be initialized during depth sensor initialization")
            exit(1)

        if not self.sensor.read():
            print("Sensor read failed during depth sensor initialization")
            exit(1)
        self.sensor.setFluidDensity(density)

    def get_depth(self):
        if self.sensor.read():
            return self.sensor.depth()
        else:
            exit(1)
