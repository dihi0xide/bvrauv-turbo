from pi.motor_control.motor_serial import MotorControl
from pi.motor_control.motor_packet import all_motors_packet
from pi.sensors.vectornav_imu import VectorNavIMU
from pi.sensors.depth_sensor import DepthSensor
from pi.sub_layers.abstract_sub import AUV

# A wrapper class for an abstract sub to control all ports (sensors and arduino)


class PinControl:
    def __init__(self, abstract_sub, motor_port, imu_port, imu_baud, depth_bus):
        """
        A wrapper class for an AUV (abstract sub) to access sensors and motors, using
        ports and I2C
        """
        self.sub = abstract_sub
        self.arduino = MotorControl(motor_port)
        self.imu = VectorNavIMU(imu_port, imu_baud)
        self.depth_sensor = DepthSensor(depth_bus)

    def update_data(self):
        """Update data based on current sensor readings"""
        self.sub.set_depth(self.depth_sensor.get_depth())
        self.sub.set_heading(self.imu.get_heading())
        self.sub.set_heading_diff(self.imu.angle_to(self.sub.wanted_heading))

    def update_motors(self):
        """Send a packet to update all motors"""
        self.arduino.send(all_motors_packet(self.sub.get_motors()))
