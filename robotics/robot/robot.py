import numpy as np


class Robot:
    def __init__(self,
                 wheel_radius,
                 axis_length,
                 claw_motor,
                 left_motor,
                 right_motor):
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
