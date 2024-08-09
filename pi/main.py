from sensors.vectornav_imu import VectorNavIMU
from sensors.depth_sensor import DepthSensor
from motor_control.motor_serial import MotorControl
from motor_control.pid.PID import PID
from motor_control.pid.differential_PID import DifferentialPID
from motor_control.motor_packet import MotorPacket
from motor_control.direction import direction
import time

wanted_depth = 2
wanted_speed = 0.5

vn = VectorNavIMU('/dev/ttyUSB0', 921600)
depth_sensor = DepthSensor()
motor_controller = MotorControl('/dev/ttyUSB1')
depth_pid = PID(0.6, 0.0, 0.1, wanted_depth)
# heading_pid = DifferentialPID(PID(0.5, 0, 0.1),)

def up(speed):
    return MotorPacket([(speed, direction("VFL")), (speed, direction("VFR")), (speed, direction("VBL")), (speed, direction("VBR"))])

def forward(speed):
    return MotorPacket([(speed, direction("HFL")), (speed, direction("HFR")), (speed, direction("HBL")), (speed, direction("HBR"))])
time.sleep(5)

# wanted_heading = vn.get_heading()
# heading_pid = DifferentialPID(PID(0.5, 0.0, 0.1, wanted_heading), 0.05)

while True:
    time.sleep(0.1)
    motor_controller.send(forward(wanted_speed))
    motor_controller.send(up(depth_pid.signal(depth_sensor.get_depth())))
