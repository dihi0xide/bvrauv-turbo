from sensors.vectornav_imu import VectorNavIMU
from sensors.depth_sensor import DepthSensor
from motor_control.motor_serial import MotorControl
from motor_control.pid.PID import PID
from motor_control.motor_packet import MotorPacket
from motor_control.direction import direction
import time

wanted_depth = -0.6
wanted_speed = 0

vn = VectorNavIMU('/dev/ttyUSB0', 921600)
depth_sensor = DepthSensor()
motor_controller = MotorControl('/dev/ttyUSB1')
depth_pid = PID(0.7, 0, 0.1, wanted_depth)
back_gain = 2
initial_depth = -1.6 #surface depth
def up(speed):
    return MotorPacket([(speed, direction("VFL")), (speed, direction("VFR")), (speed*back_gain, direction("VBL")), (speed*back_gain, direction("VBR"))])

def forward(speed):
    return MotorPacket([(speed, direction("HFL")), (speed, direction("HFR")), (speed, direction("HBL")), (speed, direction("HBR"))])

print('starting start sequence')
time.sleep(15)
try:
    while True:
        time.sleep(0.1)
        motor_controller.send(forward(wanted_speed))
        depth_signal = depth_pid.signal(depth_sensor.get_depth())
        if abs(depth_signal) < -1:
            depth_signal = 0.5 * (abs(depth_signal)/depth_signal)
        if abs(depth_signal) > 1:
            depth_signal = abs(depth_signal)/depth_signal
        motor_controller.send(up(depth_signal))
        print(depth_signal*-1, depth_sensor.get_depth())
except:
    print("Stopping...")
finally:
    motor_controller.send(forward(0))
    motor_controller.send(up(0))