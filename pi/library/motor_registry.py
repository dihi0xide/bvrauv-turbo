from typing import List, Callable
import numpy as np

from velocity_state import VelocityState
from logger import LogLevel

class Motor:
    
    def __init__(self, thrust_vector: np.ndarray, position: np.ndarray, set_motor: Callable, initialize: Callable):
        """
        Takes in a thrust vector, rotation quaternion (how the sub will rotate if just this motor is powered),
        and a function to set this motor. The set_motor function should take in any number in the range
        [-1, 1] (possibly excluding a deadzone, which may be defined in your AUV creation) and turn the
        motor at the appropriate power.
        """
        self.thrust_vector: np.ndarray = thrust_vector
        self.position: np.ndarray = position
        self.set = set_motor

        self.initialize = initialize





class MotorController:

    def __init__(self, *, inertia: np.ndarray, motors: List[Motor], motor_min: float = 0):
        self.inertia: np.ndarray = inertia  # the inertia tensor of the entire body
        self.motors: List[Motor] = motors   # the list of motors this sub owns
        self.motor_min: float = motor_min   # the minimum value this sub's motors can go at
        self.log: Callable = lambda str: print(f"Motor logger is not set --- {str}")

        self.last_command = VelocityState()

    def overview(self) -> None:
        self.log("---Motor controller overview---")
        self.log(f"Motor min: {self.motor_min}")
        self.log(f"Inertia tensor:\n{self.inertia}")
        self.log(f"{len(self.motors)} motors connected")

    def initialize(self) -> None:
        self.log("Initializing motors...")
 
        problems = 0
        for motor in self.motors:
            problems += motor.initialize()
        
        level = LogLevel.INFO if problems == 0 else LogLevel.WARNING

        self.log(f"Motors initalized with {problems} problem{"" if problems==1 else "s"}", level=level)


#     def register_motor(self, thrust_vector: np.ndarray, rotation_vector: np.ndarray, set_motor: function) -> int:
#         """
#         Register a motor with the key `key`, with the given thrust vector.\n
#         The `set_motor` function should take in one float in the range [-1, 1], 
#         excluding (-motor_min, motor_min), EXCEPT FOR ZERO, and set this motor to that speed.\n
#         Remember that magnitude matters; if one thrust vector is twice the magnitude of another, it will be assumed it has twice the motor strength. Remember to normalize!\n
#         Rotation vector represents the rotation that turning on JUST THIS MOTOR would provide when pitch=0 yaw=0 roll=0, in [pitch, yaw, roll]. Positive is clockwise, negative is counterclockwise.\n
#         Returns the index of the motor list that this motor has been placed at; in general, you shouldn't need to access the list, but it's there just in case.
#         """

#         self.motors.append(np.concatenate([thrust_vector, rotation_vector]).reshape(-1, 1))
#         self.motor_setters.append(set_motor)
#         return len(self.motors)-1

#     def get_motor_matrix(self) -> np.ndarray:
#         return np.hstack(self.motors)

#     def motors_from_vector(self, thrust_vector: np.ndarray, rotation_vector: np.ndarray, motor_min: float) -> Union[np.ndarray, None]:
#         """
#         Create a vector of motor commands, given the vectors of wanted thrust and rotational velocities. motor_min represents the deadzone in both negative and positive.\n
#         If the given vectors cannot be achieved, `None` will be returned.\n
#         This function is expensive, whenever possible it should be precomputed.
#         """


#         b = np.concatenate([thrust_vector, rotation_vector])
#         M = self.get_motor_matrix()
#         n = M.shape[1]  # number of unknowns
        
#         # the unknown vector is u, find the sparsest solution of 
#         # M*u=b
#         # all elements of u 
#         # -must be in the range [-1, 1]
#         # -may NOT be in the range (-motor_min, motor_min), EXCEPT for 0
#         # -it CAN be 0, and 0 should be encouraged for sparsity

# a = MotorController()

def n(v):
    v2 = np.array(v)
    return v/np.linalg.norm(v2)

# a.register_motor(n([1, 1]), n([-1]))
# a.register_motor(n([-1, 1]), n([1]))
# a.register_motor(n([1, -1]), n([1]))
# a.register_motor(n([-1, -1]), n([-1]))

# import time

# mat = a.get_motor_matrix()

# g = time.perf_counter()
# c = a.motors_from_vector(np.array([0.1, 0.1]), np.array([0]), 0.1)
# e = time.perf_counter()
# print(1000*(e-g))

# # print(mat)
# # print(c)
# np.set_printoptions(suppress=True)

# print("\n---outputs---\n")
# print(c)
# print(np.round(mat @ c, decimals=10))

# # a = np.array([0, 1, 2, 3])
# # print(np.count_nonzero(a))
# # print(np.sum(np.abs(a)))

import cvxpy as cp
import numpy as np

# Example inputs (replace with actual values)
M = np.hstack(
    (np.array(n([[1], [0], [1]])),
    np.array(n([[1], [0], [-1]])),
    np.array(n([[-1], [0], [-1]])),
    np.array(n([[-1], [0], [1]])))
)


v = np.array([-0.1, 0, 0])   # Desired velocity vector

# print(M)
# print(M @ np.array([0.5, -0.2, 0.3]))
# raise Exception()
# Variables
u = cp.Variable(4)         # Motor speeds
b = cp.Variable(4, boolean=True)  # Binary variables for dead zone enforcement

# Constraints
constraints = [
    M @ u == v,              # Ensure velocity matches target
    u >= -1,                 # Physical limits
    u <= 1,                  # Physical limits
    u >= 0.1 * b - 1 + 1 * b,  # If b = 1, u >= 0.1; if b = 0, u <= 0
    u <= -0.1 * b + 1 - 1 * b, # If b = 1, u <= -0.1; if b = 0, u >= 0
]

# Objective: Minimize L1-norm to encourage sparsity
objective = cp.Minimize(cp.norm1(u))

# Problem definition
problem = cp.Problem(objective, constraints)

# Solve the problem
problem.solve()

# Results
if problem.status == cp.OPTIMAL:
    print("Optimal motor speeds:", u.value)
    print("Binary variables (b):", b.value)
else:
    print("Problem is infeasible or not solved.")

print(np.round(M @ u.value, decimals=10))