from typing import List, Callable, Optional
import numpy as np
from pulp import LpVariable, LpBinary, LpProblem, LpMinimize, lpSum, PULP_CBC_CMD, LpStatus, value # type: ignore
import time

from velocity_state import VelocityState
from logger import LogLevel



class Motor:

    class Range:
        def __init__(self, top: float, bottom: float):
            self.max = top
            self.min = bottom

    def __init__(self, thrust_vector: np.ndarray, position: np.ndarray, set_motor: Callable, initialize: Callable, upper_bound: Range, lower_bound: Range):
        self.thrust_vector: np.ndarray = thrust_vector
        self.position: np.ndarray = position
        self.set: Callable = set_motor

        self.initialize: Callable = initialize
        self.inertia_tensor: Optional[np.ndarray] = None
        self.torque_vector: Optional[np.ndarray] = None

        self.upper_bound: Motor.Range = upper_bound
        self.lower_bound: Motor.Range = lower_bound

    def set_inertia_tensor(self, inertia_tensor):
        self.inertia_tensor = inertia_tensor
        self.torque_vector = np.cross(self.position, self.thrust_vector)    


class MotorController:

    def __init__(self, *, inertia: np.ndarray, motors: List[Motor]):
        self.inertia: np.ndarray = inertia  # the inertia tensor of the entire body
        self.motors: np.ndarray = np.array(motors)   # the list of motors this sub owns
        self.log: Callable = lambda str, level=None: print(f"Motor logger is not set --- {str}")

        self.motor_matrix = np.array([])

        for motor in motors:
            motor.set_inertia_tensor(self.inertia)

        self.reset_motor_matrix()

        self.last_command = VelocityState()

    def overview(self) -> None:
        self.log("---Motor controller overview---")
        self.log(f"Inertia tensor:\n{self.inertia}")
        self.log(f"{len(self.motors)} motors connected")

    def initialize(self) -> None:
        self.log("Initializing motors...")
 
        problems = 0
        for motor in self.motors:
            problems += motor.initialize()
        
        level = LogLevel.INFO if problems == 0 else LogLevel.WARNING

        self.log(f"Motors initalized with {problems} problem{'' if problems==1 else 's'}", level=level)
    
    def reset_motor_matrix(self):
        for i, motor in enumerate(self.motors):
            new_vector = np.array([np.concatenate([motor.thrust_vector, self.inertia @ motor.torque_vector], axis=None)]).T

            if(i == 0):
                self.motor_matrix = new_vector
            else:
                self.motor_matrix = np.hstack((self.motor_matrix, new_vector))

    def solve_motor_system(self, wanted_vector, M_big=1000):
        start = time.time()
        M = self.motor_matrix
        n = M.shape[1]  # Number of variables
        
        # Create the problem
        problem = LpProblem("Motor_Solver", LpMinimize)

        # Define variables
        u = [LpVariable(f'u_{i}', -1, 1) for i in range(n)]
        abs_u = [LpVariable(f'abs_u_{i}') for i in range(n)]
        
        # Add constraints for absolute values
        for i in range(n):
            problem += abs_u[i] >= u[i]
            problem += abs_u[i] >= -u[i]

        # Add constraints for force/torque matching
        for i in range(len(wanted_vector)):
            problem += lpSum(M[i, j] * u[j] for j in range(n)) == wanted_vector[i]

        # Add constraints for motor bounds
        for i in range(n):
            motor = self.motors[i]
            bottom = motor.lower_bound
            top = motor.upper_bound

            problem += u[i] >= bottom.min
            problem += u[i] <= top.max

        # Objective: Minimize activation magnitudes
        problem += lpSum([M_big * abs_u[i] for i in range(n)])

        # Solve the problem
        status = problem.solve(PULP_CBC_CMD(msg=False))
        length = (time.time() - start) * 1000

        if LpStatus[status] == "Optimal":
            activation_vector = np.array([value(u[i]) for i in range(n)])
            self.log(
                f"Successfully solved target vector {wanted_vector} with activation vector {activation_vector} in {length} milliseconds"
            )
            return activation_vector
        else:
            self.log(
                f"Failed to solve target vector {wanted_vector} after {length} milliseconds",
                level=LogLevel.ERROR,
            )
            return None

from inertia import *

while True:
    test = MotorController(inertia=InertiaBuilder(Cuboid(10, np.array([0, 0, 0]), 5, 5, 1)).moment_of_inertia(), 
                            motors=[
                                Motor(np.array([1, 1, 0]), np.array([-1, 1, 0]), lambda _: 0, lambda _: 0, Motor.Range(1, 0.1), Motor.Range(-0.1, -1)),
                                Motor(np.array([-1, 1, 0]), np.array([1, 1, 0]), lambda _: 0, lambda _: 0, Motor.Range(1, 0.1), Motor.Range(-0.1, -1)),
                                Motor(np.array([-1, 1, 0]), np.array([-1, -1, 0]), lambda _: 0, lambda _: 0, Motor.Range(1, 0.1), Motor.Range(-0.1, -1)),
                                Motor(np.array([1, 1, 0]), np.array([1, -1, 0]), lambda _: 0, lambda _: 0, Motor.Range(1, 0.1), Motor.Range(-0.1, -1))
                                ]
                            )

    solution = test.solve_motor_system(np.array([0.1, 1, 0, 0, 0, 20]))

    print(solution)