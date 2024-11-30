import numpy as np
import time
import quaternion
import traceback
from typing import Callable

from motor_registry import MotorController
from sensor_interface import SensorInterface
from mission import Path
from logger import Logger, LogLevel

class AUV:
    def __init__(self, *, 
                 motor_controller: MotorController, # the object to control the motors with
                 sensors: SensorInterface,          # the interface for sensor data
                 pin_kill: Callable,                # an emergency kill function; should disable all motors via pins

                 logging: bool = False,             # whether to save log to file
                 console: bool = True,              # whether to print log to console
                 ):
        """
        Create a sub wrapper object.\n
        motor_controller: the object to control the motors with\n
        sensors: the interface for all sensor data\n
        pin_kill: an emergency kill function, when the library is having issues. Should manually set motors off\n
        logging: whether to save log to file\n
        console: whether to print log to console
        """
        self.motor_controller = motor_controller
        self.sensors = sensors
        self.pin_kill = pin_kill
        
        self.logger = Logger(console, logging)

        self.motor_controller.log = self.logger.create_sourced_logger("MOTOR")
        self.sensors.log = self.logger.create_sourced_logger("SENSOR")
        self.sensors.depth_sensor.log = self.logger.create_sourced_logger("DEPTH")
        self.sensors.imu.log = self.logger.create_sourced_logger("IMU")

        self.logger.log(f"Sub enabled")
        self.motor_controller.overview()
        self.sensors.overview()
        
        self.motor_controller.initialize()
        self.sensors.initialize()



    def travel_path(self, mission: Path) -> None:
        """Execute each Task in the given Path, in order, then kill the sub. Handles errors."""

        self.logger.log("Beginning path")

        try:
            for task in mission:
                self.logger.log(f"Beginning task {task.name}")
                task.update(self.motor_controller, self.sensors)

        except:
            self.logger.log(traceback.format_exc())
    
        finally:
            self.logger.log("Killing sub")
            if(not self.motor_controller.killed()):
                kill_methods = [
                ("kill", self.kill),
                # kill through sub interface, uses full library to send kill. should always work

                ("backup kill", self.pin_kill)
                # last resort, directly control pins and send kill commands. doesn't go through library
                # at all, just sends pin commands
                ]
                # when we get more kills (eg hardware kill once we connect it to raspi) add them here
            
            for method_name, method in kill_methods:
                self.logger.log(f"Attempting {method_name}...")
                method()
                if self.sensors.killed():
                    self.logger.log(f"{method_name.capitalize()} succeeded")
                    break
                else:
                    self.logger.log(f"{method_name.capitalize()} failed", level=LogLevel.ERROR)
            else:
                self.logger.log("All kills ineffective. Manual intervention required", level=LogLevel.WARNING)
            
            self.logger.end()