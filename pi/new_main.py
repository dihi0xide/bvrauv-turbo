from motor_control.pid.PID import PID
from motor_control.pid.PID_bounds import BoundedPID
from motor_control.motor_serial import MotorControl
from sensors.depth_sensor import DepthSensor
from sub import AUV
import time

# # # CONFIG # # #

wanted_depth = 0.3  # relative to beginning
wanted_speed = 0

max_speed = 1
min_speed = 0.1
update_interval = 0.1

initial_wait = 2

print_depth_debug = True

# Depth PID #
Dp = 0.6
Di = 0.0
Dd = 0.1


# # # CONFIG END # # #

sub = AUV()

print("Begun connecting devices...")
# vn = VectorNavIMU('/dev/ttyUSB0', 921600)
depth_sensor = DepthSensor()
motor_controller = MotorControl('/dev/ttyUSB0')
print("Finished connecting!")

print("Begun generating PIDs...")
depth_pid = BoundedPID(PID(Dp, Di, Dd, wanted_depth), max_speed, min_speed)
print("Finished PIDs!")

print("Begun calibrating depth sensor...")
# depth_sensor.calibrate(10)
print("Finished calibrating!")

print("Begun connecting to sub framework...")
# sub.connect_depth_sensor(depth_sensor, use_relative=True)
# sub.begin_depth_pid(depth_pid)
sub.connect_arduino(motor_controller)
print("Finished connecting!")

initial_time = time.time()
for seconds in range(initial_wait, 0, -1):
    print(seconds, "seconds to startup")
    time.sleep(1)

while True:
    sub.update_motors(depth_debug=print_depth_debug)
    time.sleep(update_interval)
