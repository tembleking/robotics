from typing import Tuple

from robotics.robot.robot import Robot
from robotics.sensors.camera import Camera
from robotics.geometry import Point, Location
from robotics.robot.obstacle_trajectory_speed_generator import ObstacleTrajectorySpeedGenerator
from robotics.robot.trajectory_generator import TrajectoryGenerator

class RobotFinderSpeedGenerator:
    def __init__(self, is_white_map: bool, camera: Camera, obstacle_speed_generator: ObstacleTrajectorySpeedGenerator):
        self.camera = camera
        self._is_white_map = is_white_map
        self._robot_found = False
        self._obstacle_trajectory_speed_generator = obstacle_speed_generator

    def get_speed(self, current_location: Location) -> Tuple[float, float]:
        if self._robot_found:
            return self._obstacle_trajectory_speed_generator.get_speed(current_location)
        robot_position = self.camera.get_homography_robot_position()
        if robot_position is None:
            return 0, 0
        self._robot_found = True
        new_destination = self._get_destination_based_on_map(robot_position)
        self._obstacle_trajectory_speed_generator.trajectory_generator.change_destination(new_destination)
        return self._obstacle_trajectory_speed_generator.get_speed(current_location)

    def _get_destination_based_on_map(self, robot_position: str) -> Point:
        offset = 3 if self._is_white_map else 0
        if robot_position == "left":
            return Point(0 + offset, 6)
        else:
            return Point(3 + offset, 6)
