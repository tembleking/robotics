import math
from robotics.actuators import brickpi3


class Motor:
    def __init__(self, BP: brickpi3.BrickPi3, connected_port: int, motor_name: str):
        self.BP = BP
        self.connected_port = connected_port
        self.motor_name = motor_name
        self.BP.reset_motor_encoder(connected_port)

    def set_speed(self, angular_speed: float):
        degrees = math.degrees(angular_speed)
        print('motor: %s, settings speed (degrees): %s' % (self.motor_name, degrees))
        self.BP.set_motor_dps(self.connected_port, degrees)

    def get_last_angle(self) -> float:
        angle = self.BP.get_motor_encoder(self.connected_port)
        self.BP.reset_motor_encoder(self.connected_port)
        return math.radians(angle)
