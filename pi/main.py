from sensors.vectornav_imu import VectorNavIMU
from sensors.depth_sensor import DepthSensor
from motor_control.motor_serial import MotorControl
from motor_control.pid.PID import PID
from motor_control.motor_packet import MotorPacket
from motor_control.direction import direction

wanted_depth = 2
wanted_speed = 10

vn = VectorNavIMU('/dev/ttyUSB0', 921600)
depth_sensor = DepthSensor()
motor_controller = MotorControl('/dev/ttyUSB1')
depth_pid = PID(0.6, 0.1, 0.1, wanted_depth)

def up(speed):
    return MotorPacket([(speed, direction("VFL")), (speed, direction("VFR")), (speed, direction("VBL")), (speed, direction("VBR"))])

def forward(speed):
    return MotorPacket([(speed, direction("HFL")), (speed, direction("HFR")), (speed, direction("HBL")), (speed, direction("HBR"))])

while True:
    motor_controller.send(forward(wanted_speed))
    motor_controller.send(up(depth_pid.signal(depth_sensor.get_depth())))
