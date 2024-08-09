from sensors.fake_depth_sensor import FakeDepthSensor
from pi.motor_control.direction import key_from_direction


def interpret_command(command):
    split_commands = command.split(':')
    split_commands.pop()
    individual_numbers = [comm.split(',') for comm in split_commands]
    return [(float(mag), int(motor)) for mag, motor in individual_numbers]


class TestEnvironment:
    def __init__(self, fake_depth_sensor):
        self.depth = 0
        self.vertical_motors = 0
        # TODO this assumes all motors are equal; no they're not!
        # TODO rotation! quaternions!

        self.vertical_velocity = 0
        # TODO horizontal motion should be a 3d vector

        self.damping = 0.1  # how much the water damps motion every tick (multiplicative)

        self.depth_sensor = fake_depth_sensor

    def run_command(self, command):
        # TODO this is bad it only checks vertical speed
        # TODO fix this. do it
        commands = interpret_command(command)
        vertical_speed = 0
        for mag, motor in commands:
            key = key_from_direction(motor)
            if key[0] == "V":  # TODO this is disgusting code
                self.vertical_motors = mag
                break

    def tick(self):
        self.vertical_velocity *= self.damping
        self.depth += self.vertical_velocity
        self.vertical_velocity += self.vertical_motors
