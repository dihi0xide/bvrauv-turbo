import numpy as np
from abc import ABC, abstractmethod

# The abstract interface for interacting with the hardware; this should be
# extended and registered into the auv

class ImuInterace(ABC):

    def __init__(self):
        super().__init__()
        self.log = lambda str: print(str)

    @abstractmethod
    def get_accelerations() -> np.ndarray:
        pass

    @abstractmethod
    def get_rotation() -> np.quaternion:
        pass

    @abstractmethod
    def initialize() -> None:
        pass

class DepthInterface(ABC):

    def __init__(self):
        super().__init__()
        self.log = lambda str: print(str)
    
    @abstractmethod
    def get_depth() -> float:
        pass

    @abstractmethod
    def initialize() -> None:
        pass

class SensorInterface:

    def __init__(self, *, imu: ImuInterace, depth: DepthInterface):
        self.imu: ImuInterace = imu
        self.depth: DepthInterface = depth
        self.log = lambda str: print(str)

    def initialize(self) -> None:
        self.imu.initialize()
        self.depth.initialize()