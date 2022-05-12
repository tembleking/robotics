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

    def get_speed(self, current_location: Location):
        next_relative_location = self._get_next_relative_location(current_location)
        if next_relative_location is None:
            return None

        if self._has_arrived(next_relative_location):
            self._mark_point_as_visited()
            return 0, 0

        if self._has_arrived_angle(next_relative_location):
            if self.obstacle_detector.obstacle_detected():
                self.last_point_distance = math.inf
                self.last_point_angle = math.inf
                self._has_arrived_angle_var = False
                print('[TrajectoryController]: obstacle detected')
                self.trajectory_generator.mark_wall_ahead()
                return 0, 0

        return self._get_next_velocities(next_relative_location)

    def _has_arrived_angle(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        self._has_arrived_angle_var = angle_to_arrive > self.last_point_angle or self._has_arrived_angle_var or angle_to_arrive == 0 or self.last_point_angle == 0
        self.last_point_angle = angle_to_arrive
        return self._has_arrived_angle_var

    def _has_arrived_distance(self, next_relative_location: Location) -> bool:
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = distance_to_arrive > self.last_point_distance and distance_to_arrive < 0.2
        self.last_point_distance = distance_to_arrive
        return has_arrived

    def _has_arrived(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = self._has_arrived_angle(next_relative_location) and self._has_arrived_distance(
            next_relative_location)
        print(
            '[TrajectoryController]: distance_to_arrive: %s, last_point_distance: %s, angle_to_arrive=%s, has_arrived=%s' % (
                distance_to_arrive, self.last_point_distance, angle_to_arrive, has_arrived))
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

        print('[TrajectoryController]: current location seen from world: %s %s' % (
            self._position_to_cell(current_location.origin), current_location))
        if current_location is None:
            return None

        world_seen_from_current_location = current_location.inverse()
        next_location_from_current_location = next_point.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location

    def _get_next_velocities(self, next_relative_location: Location):
        if next_relative_location.angle_radians() > 0.025:
            return 0, 0.25
        if next_relative_location.angle_radians() < -0.025:
            return 0, -0.25
        return 0.1, 0
