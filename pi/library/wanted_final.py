from auv import AUV



#TODO
# -- redo velocity state
# -- inertia builder
# -- tasks
# -- test environment


# this isn't real code. i just want this to be what the mission control would be once
# the library is finished

class DepthSensor(DepthInterface):

    def __init__():
        # initialize, connect to sensor, etc...
        pass

    @override
    def get_depth():
        return 0.5

class IMU(ImuInterface):
    def __init__():
        # initialize, connect to sensor, etc...
        pass

    @override
    def get_accelerations():
        return [0.0, 0.0, 0.0]

    @override
    def get_direction():
        return np.quaternion([1, 0, 0, 0])

def emergency_kill():
    pass

def set_motor(id, speed):
    set_pin(id, speed)

anchovy = AUV(
    motor_controller=MotorController(
        
        inertia=InertiaBuilder(
            inertia.HollowCylinder(
                mass=5, 
                radius=3,
                height=10, 
                center=np.array([0,0,0]),
                facing=np.array([0,1,0])
                ),
            
            inertia.HollowCylinder(
                mass=2, 
                radius=3,
                height=10, 
                center=np.array([0,0,5]),
                facing=np.array([0,1,0])
                ),

            inertia.HollowCylinder(
                mass=2, 
                radius=3,
                height=10, 
                center=np.array([0,0,-5]),
                facing=np.array([0,1,0])
                ),

            inertia.SolidRectangle(
                mass=3, 
                size=np.array([12, 2.5, 12]), 
                center=np.array([0,0,0]),
                normal=np.array([0,1,0])
                ),
        ).build(),

        motors=[
            Motor(np.array([1, 0, 1]), np.array([5, 0, 5]), lambda x: set_motor(0, x)),
            Motor(np.array([1, 0, -1]), np.array([-5, 0, 5], lambda x: set_motor(1, x))),
            Motor(np.array([1, 0, 1]), np.array([-5, 0, -5], lambda x: set_motor(2, x))),
            Motor(np.array([1, 0, -1]), np.array([5, 0, -5], lambda x: set_motor(3, x))),

            Motor(np.array([0, 1, 0]), np.array([4, 0, 4], lambda x: set_motor(4, x))),
            Motor(np.array([0, 1, 0]), np.array([4, 0, -4], lambda x: set_motor(5, x))),
            Motor(np.array([0, 1, 0]), np.array([-4, 0, -4], lambda x: set_motor(6, x))),
            Motor(np.array([0, 1, 0]), np.array([-4, 0, 4], lambda x: set_motor(7, x))),
        ],
        motor_min=0.1,
    ),

    sensors=SensorInterface(
        imu=IMU(),
        depth_sensor=DepthSensor()
    )

    logging=False,      # whether to log to a file
    console=False,      # whether to log to the console

    pin_kill=emergency_kill   # directly access pins to disable all motors, for emergency use
)


mission = Path(
    Calibrate(),                                        # calibrate all sensors
    BeginHoldDepth(depth=10),                           # begin to hold depth, initial depth of 10
        
    TravelVectorTimed(np.array([1, 0, 0]), seconds=10), # travel on the x axis for 10 seconds

    HoldPosition(wait=3),                               # once it's arrived, realign until still for 3 secs

    Spin(spins=3),                                      # spin three times

    TravelVectorTimed(np.array([2, 0, 0]), seconds=5),  # travel on the x axis twice as fast for 5 secs

    TravelVectorTimed(np.array([0, 4, 0]), seconds=5)   # travel straight up for 5 secs at max speed
)                                                       # end of path, motors killed

anchovy.travel_path(mission)


# try:
#     anchovy.travel_path(mission)
#     log_print("Path travelled succesfully.")
# except:
#     log_print(f"Error during path; saving logs to {current_file}")

# finally:

#     if anchovy.killed():
#         log_print("Sub is killed, ending.")
    
#     else:
#         kill_methods = [
#             ("kill", anchovy.kill),
#             # kill through sub interface, uses full library to send kill. should always work

#             ("emergency kill", anchovy.emergency_kill), 
#             # in case that doesn't work (maybe through some library bug), try just 
#             # directly making a packet object and sending it

#             ("backup kill", anchovy.backup_kill)
#             # last resort, directly control pins and send kill commands. doesn't go through library
#             # at all, just sends pin commands
#             ]
        
#         for method_name, method in kill_methods:
#             log_print(f"Attempting {method_name}...")
#             if method():
#                 log_print(f"{method_name.capitalize()} succeeded.")
#                 break
#             else:
#                 log_print(f"{method_name.capitalize()} failed.")
#         else:
#             log_print("All kills ineffective. Manual intervention required.")

