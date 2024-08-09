from pi.port_controller import PortController
from motor_packet import MotorPacket
from direction import directions

# A wrapper class to communicate with the arduino motor controller

# TODO consider output of arduino; events maybe?
# this is a good idea; but probably not at this (the 2024) competition.
# if future people are looking at this, use asyncio to make an event system


class MotorControl:
    def __init__(self, port):

        self.port = port
        self.serial = PortController(port)

    def send(self, packet, debug=False):
        """Send a packet of commands over this port"""
        packet_string = packet.toString()
        self.serial.write(packet_string)
        if debug:
            print(packet_string)

    def kill(self):
        commands = []
        for direction in directions.values():
            commands.append((0, direction))
        self.serial.write(MotorPacket(commands).toString())
