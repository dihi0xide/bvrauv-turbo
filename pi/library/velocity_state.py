from typing import TypeVar, Type
import numpy as np

FV = TypeVar("FV", bound="VelocityState")

class VelocityState:
    
    def from_vector(self, vec: np.ndarray) -> FV:
        """
        Create a velocity state to follow a given 3d vector. Does not affect rotation.\n
        The magnitude controls the speed of the sub. Refer to your motor definitions for the speeds.
        """
        scaled_vec = (vec / vec.max()) * speed
        # this will bring the highest motor to 1 and scale all other motors equally, and then
        # multiply them all by the set speed
        
        return VelocityState( 
            forward = scaled_vec[0],
            vertical = scaled_vec[1],
            sideway = scaled_vec[2]
        )

    def from_quaternion() -> FV:
        # TODO learn quaternions!
        pass

    def __init__(self, *, 
    roll:float=0, 
    pitch:float=0, 
    yaw:float=0, 
    forward:float=0, 
    sideway:float=0, 
    vertical:float=0
    ) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.forward = forward
        self.sideway = sideway
        self.vertical = vertical

    
    def dead(self) -> bool:
        return (
            self.roll == 0
            and self.pitch == 0 
            and self.yaw == 0 
            and self.forward == 0 
            and self.sideway == 0 
            and self.vertical == 0
        )