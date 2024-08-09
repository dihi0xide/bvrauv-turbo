import pi.ms5837 as ms5837
import time


# wrapper class to get data out of the depth sensor
class DepthSensor:
    def __init__(self, bus=1, density=ms5837.DENSITY_FRESHWATER):
        self.sensor = ms5837.MS5837_30BA(bus)
        self.initial = 0
        self.initialize_sensor(density)
        # default I2C bus is 1 (pi 3)

    def initialize_sensor(self, density=ms5837.DENSITY_FRESHWATER):
        if not self.sensor.init():
            print("Sensor could not be initialized during depth sensor initialization")
            raise ConnectionError()

        if not self.sensor.read():
            print("Sensor read failed during depth sensor initialization")
            raise ConnectionError()

        self.sensor.setFluidDensity(density)

    def get_depth(self):
        if self.sensor.read():
            return self.sensor.depth()
        else:
            print("Depth sensor died!")
            exit(1)  # TODO error handling! seriously! this is bad!

    def get_relative_depth(self):
        return self.get_depth() - self.initial

    def calibrate(self, sample_count, interval=0.1):
        """Zero the depth sensor (use get_relative_depth() to take calibration into account)"""
        samples = []
        for i in range(sample_count):
            samples.append(self.get_depth())
            time.sleep(interval)
        self.initial = sum(samples) / sample_count
