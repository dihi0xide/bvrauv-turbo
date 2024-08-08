import time

from pi.port_controller import PortController
import vnpy


# wrapper class to get data out of the imu
class VectorNavIMU:
    def __init__(self, port, baud):
        self.imu = vnpy.VnSensor()
        self.imu.connect(port, baud)

    def get_heading(self):
        """
        Heading (yaw) of the IMU. Prefer angle_to() for PIDs, as heading has a discontinuity at 180
        (180 goes straight to -180)
        """
        return self.imu.read_yaw_pitch_roll().x

    def angle_to(self, target):
        """
        Signed angle to turn to get to `target` degrees. When using a PID controller to set heading,
        prefer minimizing this (target = 0) rather than checking heading directly
        """
        yaw = self.get_heading()
        diff = target - yaw

        if abs(target - yaw) >= 180:
            sign = yaw / abs(yaw)
            abs_diff_yaw = 180 - abs(yaw)
            abs_diff_target = 180 - abs(target)
            diff = sign * (abs_diff_yaw + abs_diff_target)
        return diff

    def get_forward_speed(self):
        # TODO make this work or delete it
        # this doesnt work bc no odometry
        # # REMEMBER TO SANITIZE FIRST
        # self.port.read()
        # return 1.0
        raise NotImplementedError()


# vn = VectorNavIMU('COM3', 921600)
#
# print(vn.imu.read_model_number())
# point_to = 170