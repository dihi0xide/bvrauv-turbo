from pi.motor_control.pid.PID import PID
from pi.motor_control.pid.differential_PID import DifferentialPID
# An abstract model of a sub which contains all the needed PIDs and other low-level details
# to control the sub.


class AUV:
    def __init__(self):

        self.imu = None
        self.depth_sensor = None
        self.motor_controller = None

        self.heading_pid = None
        self.depth_pid = None

        self.heading = None
        self.depth = None
        self.wanted_heading = None
        self.wanted_speed = None

    def connect_imu(self, imu):
        self.imu = imu

    def disconnect_imu(self):
        self.imu = None

    def connect_depth_sensor(self, depth_sensor):
        self.depth_sensor = depth_sensor

    def disconnect_depth_sensor(self):
        self.depth_sensor = None

    def connect_arduino(self, motor_controller):
        self.motor_controller = motor_controller

    def disconnect_arduino(self):
        self.motor_controller = None

    def begin_depth_pid(self, Kp, Ki, Kd, wanted_depth):
        self.depth_pid = PID(Kp, Ki, Kd, wanted_depth)

    def end_depth_pid(self):
        self.depth_pid = None

    def begin_heading_pid(self, Kp, Ki, Kd, wanted_heading, motor_min):
        self.wanted_heading = wanted_heading
        self.heading_pid = DifferentialPID(PID(Kp, Ki, Kd, wanted_heading), motor_min)

    def end_heading_pid(self):
        self.heading_pid = None

    def begin_forward_movement(self, speed):
        """Sets the WANTED forward speed of the sub"""
        self.wanted_speed = speed

    def end_forward_movement(self, speed):
        self.wanted_speed = None

    # TODO this should return a list with motor labels rather than numbers, eg. ["VFL", 0.5]
    def get_motors(self):
        """
        Gets the calculated speed each of the motors should travel at, in the form \n
        (VFL, VFR, VBL, VBR, HFL, HFR, HBL, HBR) \n
        (eg. HBR = horizontal-back-right, VFL = vertical-front-left)
        """

        vertical_motors = [0] * 4
        horizontal_motors = [0] * 4

        if self.heading_pid is not None and self.imu is not None and self.wanted_heading is not None:
            heading_motors = self.heading_pid.signal(self.imu.angle_to(self.wanted_heading))  # get heading pid speed (x, y)

            for i in range(4):
                horizontal_motors[i] += heading_motors[i % 2]
                # add to diagonals



        return tuple(vertical_motors + horizontal_motors)
