from motor_control.pid.PID import PID
from motor_control.pid.PID_bounds import BoundedPID
from motor_control.motor_serial import MotorControl
from motor_control.motor_packet import MotorPacket
from motor_control.direction import direction
from sensors.vectornav_imu import VectorNavIMU
from sensors.depth_sensor import DepthSensor
from sub import AUV
import time

# # # CONFIG # # #

wanted_depth = 0.3  # relative to beginning
wanted_speed = 0.2

max_speed = 1
min_speed = 0.2
update_interval = 0.1

initial_wait = 15

print_depth_debug = True

# Depth PID #
Dp = 0.6
Di = 0.0
Dd = 0.1

# Heading PID #
Hp = 0.5
Hi = 0.0
Hd = 0.1


# # # CONFIG END # # #

sub = AUV()

print("Begun connecting devices...")
vn = VectorNavIMU('/dev/ttyUSB0', 921600)
depth_sensor = DepthSensor()
motor_controller = MotorControl('/dev/ttyUSB0')
print("Finished connecting!\n")

print("Begun calibrating depth sensor...")
depth_sensor.calibrate(50)  # 5 seconds, 50 samples
print("Finished calibrating!\n")

print("Begun calibrating IMU heading...")
initial_heading = vn.get_average_heading(50)  # 5 seconds, 50 samples
print("Finished calibrating!\n")

print("Begun generating PIDs...")
depth_pid = BoundedPID(PID(Dp, Di, Dd, wanted_depth), max_speed, min_speed)
heading_pid = BoundedPID(PID(Hp, Hi, Hd, initial_heading), max_speed, min_speed)
print("Finished PIDs!\n")

print("Begun connecting to sub framework...")
sub.connect_depth_sensor(depth_sensor)
sub.begin_depth_pid(depth_pid, use_relative=True)
sub.begin_heading_pid(heading_pid, initial_heading)
sub.connect_arduino(motor_controller)
sub.begin_forward_movement(wanted_speed)
print("Finished connecting!\n")

initial_time = time.time()
for seconds in range(initial_wait, 0, -1):
    print(seconds, "seconds to startup")
    time.sleep(1)

try:
    while True:
        sub.update_motors(depth_debug=print_depth_debug)
        time.sleep(update_interval)
finally:
    sub.motor_controller.kill()
