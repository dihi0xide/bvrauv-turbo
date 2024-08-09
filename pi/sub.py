from pi.motor_control.pid.PID import PID
from pi.motor_control.pid.differential_PID import DifferentialPID
from motor_control.motor_packet import MotorPacket
from motor_control.direction import direction
# An abstract model of a sub which contains all the needed PIDs and other low-level details
# to control the sub.


class AUV:
    def __init__(self):

        self.imu = None
        self.depth_sensor = None
        self.motor_controller = None

        self.use_relative_depth = False

        self.heading_pid = None
        self.depth_pid = None

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

    def begin_depth_pid(self, depth_pid, use_relative=False):
        self.depth_pid = depth_pid
        self.use_relative_depth = use_relative

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
    def update_motors(self, *, depth_debug=False):
        """
        Sends a command to the sub for all set pieces of this sub object
        (Depth PID, heading PID, forward speed)
        """

        vertical_all = -1
        horizontal_all = 0

        if self.depth_pid is not None and self.depth_sensor:
            depth = self.depth_sensor.get_relative_depth() if self.use_relative_depth else self.depth_sensor.get_depth()
            signal = self.depth_pid.signal(depth)
            vertical_all += signal
            if depth_debug:
                print(signal, depth)


        if self.wanted_speed is not None:
            horizontal_all += self.wanted_speed

        vertical_motors = {"VBL": vertical_all, "VBR": vertical_all, "VFL": vertical_all, "VFR": vertical_all}
        horizontal_motors = {"HBL": horizontal_all, "HBR": horizontal_all, "HFL": horizontal_all, "HFR": horizontal_all}

        if self.heading_pid is not None and self.imu is not None and self.wanted_heading is not None:
            heading_motors = self.heading_pid.signal(self.imu.angle_to(self.wanted_heading))  # get heading pid speed (x, y)
            for motor in ["HBL", "HFR"]:
                horizontal_motors[motor] += heading_motors[0]

            for motor in ["HFL", "HBR"]:
                horizontal_motors[motor] += heading_motors[1]

            # add to diagonals

        if self.motor_controller is not None:
            command_list = []

            for motor, magnitude in horizontal_motors.items():
                command_list.append((magnitude, direction(motor)))

            for motor, magnitude in vertical_motors.items():
                command_list.append((magnitude, direction(motor)))

            self.motor_controller.send(MotorPacket(command_list))
