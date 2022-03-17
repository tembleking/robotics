import time

from robotics.geometry import Location
import numpy as np


class Odometry:
    def __init__(self, left_motor, right_motor, polling_period, wheel_radius, axis_length):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.polling_period = polling_period
        self.wheel_radius = wheel_radius
        self.axis_length = axis_length

    def start(self):
        pass

    def read_speed(self):
        left_angle = self.left_motor.get_last_angle()
        left_speed = left_angle / self.polling_period

        right_angle = self.right_motor.get_last_angle()
        right_speed = right_angle / self.polling_period

        transformation_matrix = np.matrix([[self.wheel_radius / 2, self.wheel_radius / 2],
                                           [self.wheel_radius / self.axis_length,
                                            -self.wheel_radius / self.axis_length]])

        result = transformation_matrix.dot(np.matrix([[right_speed], [left_speed]]))
        return result[0, 0], result[1, 0]

    def _update(self):
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            initial_time = time.clock()

            # Read from the motors
            v, w = self.read_speed()
            # Update the Location

            end_time = time.clock()
            time.sleep(self.polling_period - (end_time - initial_time))

    def location(self) -> Location:
        pass
