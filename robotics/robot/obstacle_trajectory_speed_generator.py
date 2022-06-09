import math

from robotics.geometry import Location, Direction, Point
from robotics.robot.trajectory_generator import TrajectoryGenerator


class ObstacleTrajectorySpeedGenerator:
    def __init__(self, trajectory_generator: TrajectoryGenerator, obstacle_detector=None):
        self.trajectory_generator = trajectory_generator
        self.obstacle_detector = obstacle_detector
        self.last_point_distance = math.inf
        self.last_point_angle = math.inf
        self._has_arrived_angle_var = False
        self.started = False
        self.last_speed = (0, 0)

    def get_speed(self, current_location: Location):
        if not self.started:
            self.started = True
            self.trajectory_generator.recalculate_path()
        next_relative_location = self._get_next_relative_location(current_location)
        if next_relative_location is None:
            return None

        if self._has_arrived(next_relative_location):
            self._mark_point_as_visited()
            return self.last_speed

        if self._has_arrived_angle(next_relative_location):
            if self.obstacle_detector.obstacle_detected():
                self.last_point_distance = math.inf
                self.last_point_angle = math.inf
                self._has_arrived_angle_var = False
                self.trajectory_generator.mark_wall_ahead()
                return self.last_speed
            # return 0.1, 0

        new_speed = self._get_next_velocities(next_relative_location)
        self.last_speed = new_speed
        return new_speed

    def _has_arrived_angle(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        self._has_arrived_angle_var = angle_to_arrive > self.last_point_angle or self._has_arrived_angle_var or angle_to_arrive == 0 or self.last_point_angle == 0
        self.last_point_angle = angle_to_arrive
        return self._has_arrived_angle_var

    def _has_arrived_distance(self, next_relative_location: Location) -> bool:
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = distance_to_arrive > self.last_point_distance and distance_to_arrive < 0.1
        self.last_point_distance = distance_to_arrive
        return has_arrived

    def _has_arrived(self, next_relative_location: Location) -> bool:
        has_arrived = self._has_arrived_angle(next_relative_location) and self._has_arrived_distance(
            next_relative_location)
        return has_arrived

    def _mark_point_as_visited(self):
        self.last_point_distance = math.inf
        self.last_point_angle = math.inf
        self._has_arrived_angle_var = False
        self.trajectory_generator.mark_point_as_visited()

    def _position_to_cell(self, point: Point):
        x_cell = int(point.x * 1000 / 400)
        y_cell = int(point.y * 1000 / 400)
        return (x_cell, y_cell)

    def _get_next_relative_location(self, current_location: Location) -> Location:
        next_point = self.trajectory_generator.next_absolute_point_to_visit()
        if next_point is None:
            return None

        new_angle_with_correction = math.atan2(next_point.origin.y - current_location.origin.y,
                                               next_point.origin.x - current_location.origin.x)
        next_point = Location.from_angle_radians(next_point.origin, new_angle_with_correction)
        if current_location is None:
            return None

        world_seen_from_current_location = current_location.inverse()
        next_location_from_current_location = next_point.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location

    def _get_next_velocities(self, next_relative_location: Location):
        if next_relative_location.angle_radians() > 0.05:
            return 0, 0.25
        if next_relative_location.angle_radians() < -0.05:
            return 0, -0.25
        return 0.10, 0
