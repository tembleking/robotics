import time

from robotics.geometry import Location, Point
from robotics.robot.robot import Robot


class Controller:
    def __init__(self, speed_generators: [], robot: Robot, polling_period: float):
        self._speed_generators = speed_generators
        self._polling_period = polling_period
        self._robot = robot
        self.visited_points = []
        self._has_finished_s = False

    def start(self):
        self._robot.start_odometry()
        for speed_generator in self._speed_generators:
            while True:
                start = time.time()

                location = self._robot.location()
                print('Location:', location)
                self.visited_points.append(location)

                speed = speed_generator.get_speed(location)
                if speed is None:
                    break

                self._robot.set_speed(speed[0], speed[1])
                sleep_time = self._polling_period - (time.time() - start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # fixme(fede): Refactor this to a "after_reaching" method in the speed generators
            if not self._has_finished_s:
                location = self._robot.location()
                new_location = Location.from_angle_radians(Point(location.origin.x, location.origin.y + 0.1),
                                                           location.angle_radians())
                self._robot.set_location(new_location)
        self._robot.stop_odometry()
        self._robot.set_speed(0, 0)
