import time

from robotics.robot.robot import Robot


class Controller:
    def __init__(self, speed_generators: [], robot: Robot, polling_period: float):
        self._speed_generators = speed_generators
        self._polling_period = polling_period
        self._robot = robot
        self.visited_points = []

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
        self._robot.stop_odometry()
        self._robot.set_speed(0, 0)
