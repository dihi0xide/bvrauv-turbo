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

imu_port = '/dev/ttyUSB1'
arduino_port = '/dev/ttyUSB0'

wanted_depth = 0.5  # relative to beginning
wanted_speed = 0.2

max_speed = 1
min_speed = 0.2
update_interval = 0.1

initial_wait = 15

print_depth_debug = False

# Depth PID #
Dp = 0.6
Di = 0.0
Dd = 0.1

# Heading PID #
Hp = 0.05
Hi = 0.0
Hd = 0.03

# Mission control #
time_to_gate = 20
angle_to_hexagon = 20  # relative to initial turn
time_to_hexagon = 40
# # # CONFIG END # # #

sub = AUV()

print("Begun connecting devices...")
vn = VectorNavIMU(imu_port, 921600)
depth_sensor = DepthSensor()
motor_controller = MotorControl(arduino_port)
print("Finished connecting!\n")


print("Begun calibrating depth sensor...")
depth_sensor.calibrate(10)  # 1 second, 10 samples
print("Finished calibrating!\n")


print("Begun calibrating IMU heading...")
initial_heading = vn.get_average_heading(10)  # 1 second, 0 samples
print("Finished calibrating!\n")


print("Begun generating PIDs...")
depth_pid = PID(Dp, Di, Dd, wanted_depth)
heading_pid = PID(Hp, Hi, Hd, 0)
print("Finished PIDs!\n")

print("Begun connecting to sub framework...")
sub.connect_depth_sensor(depth_sensor)
sub.connect_imu(vn)

sub.clamp_vertical_motors(min_speed, max_speed)
sub.clamp_horizontal_motors(min_speed, max_speed)

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
    print("Beginning forward movement!")
    sub.move_time(time_to_gate, wanted_speed, depth_debug=print_depth_debug)

    print("Spinning!")
    sub.spin(3)

    print("Turning to face hexagon..")
    sub.begin_heading_pid(PID(Hp, Hi, Hd, 0), initial_heading+angle_to_hexagon)
    sub.wait_for_alignment()

    print("Moving to hexagon!")
    sub.move_time(time_to_hexagon, wanted_speed, depth_debug=print_depth_debug)

    print("Killing sub; prepare for surfacing.")

finally:
    sub.motor_controller.kill()
