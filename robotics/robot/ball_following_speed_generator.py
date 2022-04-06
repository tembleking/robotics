from typing import Tuple

from robotics.sensors.camera import Camera
from robotics.geometry import Point


class BallFollowingSpeedGenerator:
    def __init__(self, camera: Camera, area_goal: float, distance_goal: float, distance_damping: float,
                 area_damping: float):
        self.camera = camera
        self.area_goal = area_goal
        self.distance_goal = distance_goal
        self.distance_damping = distance_damping
        self.area_damping = area_damping

    def next_speed(self) -> Tuple[float, float]:
        ball_center, ball_size = self.camera.get_blob_position_and_size()
        v = self._calculate_lineal_speed(ball_size)
        w = self._calculate_angular_speed(ball_center)
        return v, w

    def _calculate_lineal_speed(self, ball_size: float) -> float:
        return self.area_damping * (self.area_goal - ball_size)

    def _calculate_angular_speed(self, ball_center: Point) -> float:
        return self.distance_damping * (self.distance_goal - ball_center.x)
