from typing import Tuple

from robotics.robot.robot import Robot
from robotics.sensors.camera import Camera
from robotics.geometry import Point, Location
from robotics.robot.obstacle_trajectory_speed_generator import ObstacleTrajectorySpeedGenerator
from robotics.robot.trajectory_generator import TrajectoryGenerator

class RobotFinderSpeedGenerator:
    def __init__(self, is_white_map: bool, camera: Camera, trajectory_generator: TrajectoryGenerator):
        self.camera = camera
        self._is_white_map = is_white_map
        self._found_exit = False
        self._obstacle_trajectory_speed_generator = None
        self._trajectory_generator = trajectory_generator

    def get_speed(self, current_location: Location) -> Tuple[float, float]:
        robot_position = self.camera.get_homography_robot_position()
        if not self._found_exit and robot_position is None:
            return 0, 0.5
        self._found_exit = True
        if self._obstacle_trajectory_speed_generator is None:
            self._trajectory_generator.change_destination(self._get_destination_based_on_map(robot_position))
            self._obstacle_trajectory_speed_generator = ObstacleTrajectorySpeedGenerator(trajectory_generator=self._trajectory_generator)
        return self._obstacle_trajectory_speed_generator.get_speed(current_location)

    def _get_destination_based_on_map(self, robot_position: str) -> Point:
        offset = 1.2 if self._is_white_map else 0
        if robot_position == "left":
            return Point(0.2 + offset, 2.6)
        else:
            return Point(1.4 + offset, 2.6)
