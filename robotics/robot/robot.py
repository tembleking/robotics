import time

import numpy as np

from robotics.geometry import Location
from robotics.robot.odometry import Odometry


class Robot:
    def __init__(self,
                 odometry: Odometry,
                 wheel_radius,
                 axis_length,
                 claw_motor,
                 left_motor,
                 right_motor):
        self.odometry = odometry
        self.wheel_radius = wheel_radius
        self.axis_length = axis_length
        self.direct_control_matrix = np.matrix(
            [
                [self.wheel_radius / 2, self.wheel_radius / 2],
                [self.wheel_radius / self.axis_length, -self.wheel_radius / self.axis_length]
            ])
        self.inverse_control_matrix = np.linalg.inv(self.direct_control_matrix)
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.claw_motor = claw_motor

    def set_speed(self, v, w):
        print('robot setting speed(v: %s, w: %s)' % (v, w))
        angular_speed = np.dot(self.inverse_control_matrix, np.array([v, w]))

        self.left_motor.set_speed(angular_speed[0, 1])
        self.right_motor.set_speed(angular_speed[0, 0])

    def start_odometry(self):
        self.odometry.start()

    def stop_odometry(self):
        self.odometry.stop()

    def location(self) -> Location:
        return self.odometry.location()

    def try_retrieve_ball(self):
        self._open_claws()
        time.sleep(5)
        self.set_speed(0.06, 0.01)
        time.sleep(1.5)
        self.set_speed(0, 0)
        self._close_claws()
        time.sleep(5)

    def _open_claws(self):
        self.claw_motor.set_position(-180)

    def _close_claws(self):
        self.claw_motor.set_position(0)

    def set_location(self, new_location):
        self.odometry.set_location(new_location)
