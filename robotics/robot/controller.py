import time

from robotics.geometry import Location, Point
from robotics.robot.final_trajectory_speed_generator import FinalTrajectorySpeedGenerator
from robotics.robot.hardcoded_speed_generator import HardcodedSpeedGenerator
from robotics.robot.obstacle_trajectory_speed_generator import ObstacleTrajectorySpeedGenerator
from robotics.robot.robot import Robot
from robotics.sensors.sonar import Sonar


class Controller:
    def __init__(self, speed_generators: [], robot: Robot, sonar: Sonar, polling_period: float):
        self._speed_generators = speed_generators
        self._polling_period = polling_period
        self._robot = robot
        self._sonar = sonar
        self.visited_points = []
        self._has_finished_s = False

    def start(self):
        self._robot.start_odometry()
        for speed_generator in self._speed_generators:
            print("using new controller")
            if isinstance(speed_generator, ObstacleTrajectorySpeedGenerator):
                    self._robot.odometry.disable_gyro()
            else:
                self._robot.odometry.enable_gyro()
            while True:
                start = time.time()

                location = self._robot.location()
                # print('Location:', location)
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
                self._has_finished_s = True
                location = self._robot.location()
                self._robot.set_speed(0,0)
                new_x = self._sonar.distance() if location.origin.x < 1 else 2.8 - self._sonar.distance()
                new_location = Location.from_angle_radians(Point(new_x, location.origin.y + 0.1),
                                                           location.angle_radians())
                self._robot.set_location(new_location)
        self._robot.stop_odometry()
        self._robot.set_speed(0, 0)
