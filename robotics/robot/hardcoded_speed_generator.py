import math

from robotics.geometry import Location, Point, Direction


class HardcodedSpeedGenerator:
    def __init__(self):
        self._trajectory_white = [
            Location.from_angle_degrees(Point(0.6, 2.6), -90),
            Location.from_angle_degrees(Point(0.6, 2.6), 180),
            Location.from_angle_degrees(Point(0.6, 1.8), 0),
            Location.from_angle_degrees(Point(0.6, 1.0), 180),
        ]
        self._trajectory_black = [
            Location.from_angle_degrees(Point(2.2, 2.6), -90),
            Location.from_angle_degrees(Point(2.2, 2.6), 0),
            Location.from_angle_degrees(Point(2.2, 1.8), 180),
            Location.from_angle_degrees(Point(2.2, 1.0), 0),
        ]
        self._speeds = [
            (0.1, 0),
            (0, -0.5),
            (0.1, 0.25),
            (0.1, -0.25),
        ]

    def get_speed(self, current_location: Location):
        next_relative_location = self._get_next_relative_location(current_location)
        if next_relative_location is None:
            return None

        if self._has_arrived(next_relative_location):
            self._pop_point_to_visit(current_location)
            self._speeds.pop(0)

        return self._current_speed(current_location)

    def _get_next_relative_location(self, current_location: Location) -> Location:
        next_point = self._get_next_absolute_point_to_visit(current_location)
        if next_point is None:
            return None

        world_seen_from_current_location = current_location.inverse()
        next_location_from_current_location = next_point.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location

    def _get_next_absolute_point_to_visit(self, current_location: Location) -> Location:
        if current_location.origin.x < 1.4:
            return self._trajectory_white[0]
        else:
            return self._trajectory_black[0]

    def _pop_point_to_visit(self, current_location: Location):
        if current_location.origin.x < 1.4:
            self._trajectory_white.pop(0)
        else:
            self._trajectory_black.pop(0)

    def _has_arrived(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        return angle_to_arrive < math.radians(0.01) and distance_to_arrive < 0.02

    def _current_speed(self, current_location: Location) -> (float, float):
        if len(self._speeds) == 0:
            return None

        if current_location.origin.x < 1.4:
            return self._speeds[0]
        else:
            v, w = self._speeds[0]
            return v, -w
