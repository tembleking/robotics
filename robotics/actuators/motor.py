import math
from robotics.actuators import brickpi3


class Motor:
    def __init__(self, BP: brickpi3.BrickPi3, connected_port: int, motor_name: str, motor_limit_dps=0):
        self.BP = BP
        self.connected_port = connected_port
        self.motor_name = motor_name
        self.BP.reset_motor_encoder(connected_port)
        self.BP.set_motor_limits(connected_port, dps=motor_limit_dps)
        self._last_angle = 0

    def set_speed(self, angular_speed: float):
        degrees = math.degrees(angular_speed)
        degrees = min(500, degrees)
        print('motor: %s, setting speed (degrees): %s, connected_port: %s' %
              (self.motor_name, degrees, self.connected_port)
              )
        self.BP.set_motor_dps(self.connected_port, degrees)

    def get_last_angle(self) -> float:
        angle_read = self.BP.get_motor_encoder(self.connected_port)
        new_angle = angle_read - self._last_angle
        self._last_angle = angle_read
        return math.radians(new_angle)

    def set_position(self, degree_angles: float):
        self.BP.set_motor_position(self.connected_port, degree_angles)
