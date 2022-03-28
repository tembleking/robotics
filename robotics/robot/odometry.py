import math
import time
from multiprocessing import Lock, Process, Value

from robotics.geometry import Location, Point
import numpy as np


class Odometry:
    def __init__(self, left_motor, right_motor, polling_period, wheel_radius, axis_length):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.polling_period = polling_period
        self.wheel_radius = wheel_radius
        self.axis_length = axis_length
        self.location_lock = Lock()
        self.finished = Value('b')
        self.finished.value = False
        self.inner_process = Process(target=self.__update)
        self.x = Value('f')
        self.y = Value('f')
        self.th = Value('f')

    def start(self):
        self.inner_process.start()

    def stop(self):
        self.finished.value = True
        self.inner_process.join()

    def read_speed(self):
        left_speed = self.left_motor.get_last_angle() / self.polling_period
        right_speed = self.right_motor.get_last_angle() / self.polling_period

        transformation_matrix = np.matrix([[self.wheel_radius / 2, self.wheel_radius / 2],
                                           [self.wheel_radius / self.axis_length,
                                            -self.wheel_radius / self.axis_length]])

        result = transformation_matrix.dot(np.matrix([[right_speed], [left_speed]]))
        return result[0, 0], result[1, 0]

    def location(self) -> Location:
        with self.location_lock:
            return Location.from_angle_radians(Point(self.x.value, self.y.value), self.th.value)

    def __update(self):
        while not self.finished.value:
            initial_time = time.time()

            # Read from the motors
            v, w = self.read_speed()
            # Update the Location
            with self.location_lock:
                # TODO: We could calculate the new location outside of the lock and then assign it
                self.x.value, self.y.value, self.th.value = self.__get_new_location_from_speed(self.x.value,
                                                                                               self.y.value,
                                                                                               self.th.value,
                                                                                               v, w)

            end_time = time.time()
            sleep_time = self.polling_period - (end_time - initial_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def __get_new_location_from_speed(self, x, y, th, v, w) -> (float, float, float):
        distance_increment = v * self.polling_period
        angle_increment = w * self.polling_period
        x_increment = distance_increment * math.cos(th + (angle_increment / 2))
        y_increment = distance_increment * math.sin(th + (angle_increment / 2))

        return x_increment + x, y_increment + y, angle_increment + th
