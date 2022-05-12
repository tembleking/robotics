from typing import Tuple

from robotics.robot.robot import Robot
from robotics.sensors.camera import Camera
from robotics.geometry import Point, Location


class BallFollowingSpeedGenerator:
    def __init__(self, camera: Camera, robot: Robot, area_goal: float, distance_goal: float, distance_damping: float,
                 area_damping: float):
        self.camera = camera
        self.robot = robot
        self.area_goal = area_goal
        self.distance_goal = distance_goal
        self.distance_damping = distance_damping
        self.area_damping = area_damping
        self.last_ball_position = 1

    def get_speed(self, _: Location) -> Tuple[float, float]:
        ball_center, ball_size = self.camera.get_blob_position_and_size()
        if ball_center is None:
            return 0, 0.5 * self.last_ball_position

        print('[BallFollowingSpeedGenerator]: Ball center: %s, Ball size: %s' % (ball_center, ball_size))
        v = self._calculate_lineal_speed(ball_size)
        w = self._calculate_angular_speed(ball_center)

        if abs(v) < 0.01 and abs(w) < 0.03:
            self.robot.set_speed(0, 0)
            self.robot.try_retrieve_ball()
            if self.camera.is_ball_within_claws():
                return None

        return v, w

    def _calculate_lineal_speed(self, ball_size: float) -> float:
        return self.area_damping * (self.area_goal - ball_size)

    def _calculate_angular_speed(self, ball_center: Point) -> float:
        offset = self.distance_goal - ball_center.x
        if offset < 0:
            self.last_ball_position = -1
        else:
            self.last_ball_position = 1

        return self.distance_damping * offset
