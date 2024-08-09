from pi import ms5837 as ms5837
from pi.sensors.depth_sensor import DepthSensor


class FakeDepthSensor(DepthSensor):
    def __init__(self, initial):
        super().__init__()
        self.depth = initial

    def initialize_sensor(self, density=ms5837.DENSITY_FRESHWATER):
        pass

    def get_depth(self):
        return self.depth

    def set_depth(self, depth):
        self.depth = depth
