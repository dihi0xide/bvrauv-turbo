from motor_control.pid.PID import PID
from motor_control.pid.differential_PID import DifferentialPID
from motor_control.pid.PID_bounds import BoundedPID
from motor_control.motor_packet import MotorPacket
from motor_control.direction import direction, directions
from collections import deque
import time
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

        self.vertical_min = None
        self.vertical_max = None

        self.horizontal_min = None
        self.horizontal_max = None

        self.set_motors = {key: None for key, _ in directions}

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

    def begin_heading_pid(self, pid, wanted_heading):
        self.wanted_heading = wanted_heading
        # self.heading_pid = DifferentialPID(PID(Kp, Ki, Kd, wanted_heading), motor_min)
        self.heading_pid = pid

    def end_heading_pid(self):
        self.heading_pid = None

    def begin_forward_movement(self, speed):
        """Sets the WANTED forward speed of the sub"""
        self.wanted_speed = speed

    def end_forward_movement(self):
        self.wanted_speed = None

    def clamp_horizontal_motors(self, horizontal_min, horizontal_max):
        self.horizontal_min = horizontal_min
        self.horizontal_max = horizontal_max

    def clamp_vertical_motors(self, vertical_min, vertical_max):
        self.vertical_min = vertical_min
        self.vertical_max = vertical_max

    def set_motor(self, motor, speed):
        # motor = (eg. 'VFL')
        self.set_motors[motor] = speed

    def update_over_time(self, seconds, depth_debug=False, update_interval=0.1):
        start = time.time()
        while seconds > time.time() - start:
            self.update_motors(depth_debug=depth_debug)
            time.sleep(update_interval)

    def move_time(self, seconds, speed, depth_debug=False, update_interval=0.1):
        self.wanted_speed = speed
        self.update_over_time(seconds, depth_debug, update_interval)
        self.end_forward_movement()

    def spin(self, num_spins=1, update_interval=0.01):
        num_degrees = 0
        wanted_degrees = 360*num_spins
        self.end_forward_movement()
        self.end_heading_pid()

        self.set_motor('HFL', 1)
        self.set_motor('HBL', 1)

        prev_angle = self.imu.get_heading()
        while wanted_degrees >= num_degrees:
            num_degrees += self.imu.angle_to(prev_angle)
            prev_angle = self.imu.get_heading()
            self.update_motors()
            time.sleep(update_interval)

    def wait_for_alignment(self, threshold=10, sample_size=30, sample_interval=0.1):
        # wait for average heading to be within `threshold` degrees. takes `sample_size` samples,
        # and waits `sample_interval` seconds between each sample
        if self.heading_pid is None:
            raise RuntimeError

        samples = deque()

        average = -1
        while average == -1 or average > threshold:
            angle = abs(self.imu.angle_to(self.wanted_heading))
            samples.appendleft(angle)

            if len(samples) >= sample_size:
                samples.pop()
                average = sum(samples) / sample_size

            time.sleep(sample_interval)

    # TODO this should return a list with motor labels, eg. ["VFL", 0.5]
    def update_motors(self, *, depth_debug=False):
        """
        Sends a command to the sub for all set pieces of this sub object
        (Depth PID, heading PID, forward speed)
        """

        vertical_all = 0
        horizontal_all = 0

        if self.depth_pid is not None and self.depth_sensor:
            depth = self.depth_sensor.get_relative_depth() if self.use_relative_depth else self.depth_sensor.get_depth()
            signal = self.depth_pid.signal(depth)
            vertical_all += signal
            if depth_debug:
                #print(signal, depth)
                #print(depth, self.depth_sensor.get_depth(), self.depth_sensor.initial)
                pass
        if self.wanted_speed is not None:
            horizontal_all += self.wanted_speed

        motors = {
            "VBL": vertical_all,
            "VBR": vertical_all,
            "VFL": vertical_all,
            "VFR": vertical_all,
            "HBL": horizontal_all,
            "HBR": horizontal_all,
            "HFL": horizontal_all,
            "HFR": horizontal_all
        }

        if self.heading_pid is not None and self.imu is not None and self.wanted_heading is not None and not self.spin:
            heading_signal = self.heading_pid.signal(self.imu.angle_to(self.wanted_heading))  # get heading pid speed (x, y)
            heading_motors = ["HFR", "HBR"] if heading_signal >= 0 else ["HFL", "HBL"]
            for motor in heading_motors:
                motors[motor] += abs(heading_signal)
            # print(heading_signal, self.imu.angle_to(self.wanted_heading), self.imu.get_heading())

        for motor, magnitude in self.set_motors:
            if magnitude is not None:
                motors[motor] = magnitude

        for motor, magnitude in motors:
            horizontal = motor[0] == 'H'
            motor_min = self.horizontal_min if horizontal else self.vertical_min
            motor_max = self.horizontal_max if horizontal else self.vertical_max

            if motor_max is not None and magnitude > motor_max:
                motors[motor] = motor_max
            if motor_min is not None and magnitude < motor_min:
                motors[motor] = motor_min

        if self.motor_controller is not None:
            command_list = [(magnitude, direction(motor)) for motor, magnitude in motors.items()]

            self.motor_controller.send(MotorPacket(command_list))
