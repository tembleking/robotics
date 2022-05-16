import math

from robotics.geometry import Point, Location
from robotics.robot.map import Map
from robotics.robot.robot import Robot


class TrajectoryGenerator:
    def __init__(self, robot: Robot, map: Map, destination: Point):
        self.robot = robot
        self.map = map
        self.destination = destination
        self._path = self._calculate_path()

    def next_absolute_point_to_visit(self) -> Location:
        if len(self._path) == 0:
            return None
        return self._path[0]

    def _cell_to_point(self, cell_x, cell_y) -> Point:
        size_in_meters = self.map.sizeCell() / 1000
        return Point(cell_x * size_in_meters + size_in_meters / 2,
                     cell_y * size_in_meters + size_in_meters / 2)

    def mark_point_as_visited(self):
        if len(self._path) > 0:
            self._path.pop(0)

    def _calculate_path(self):
        location = self.robot.location()
        current_cell = self._position_to_cell(location.origin)
        cell_path = self.map.findPath(current_cell, (self.destination.x, self.destination.y))
        print('Path: %s' % cell_path[1:])
        point_path = [self._cell_to_point(cell[0], cell[1]) for cell in cell_path]

        location_path = []
        for i in range(1, len(point_path)):
            location_path.append(
                Location.from_angle_radians(point_path[i],
                                            math.atan2(point_path[i].y - point_path[i - 1].y,
                                                       point_path[i].x - point_path[i - 1].x))
            )
        print('Location Paths: %s' % location_path)

        return location_path

    def recalculate_path(self):
        self._path = self._calculate_path()

    def _position_to_cell(self, point: Point):
        x_cell = int(point.x * 1000 / self.map.sizeCell())
        y_cell = int(point.y * 1000 / self.map.sizeCell())
        return (x_cell, y_cell)

    def mark_wall_ahead(self):
        location = self.robot.location()
        neighbor = self._angle_degrees_to_neighbor(location.angle_degrees())
        cell = self._position_to_cell(location.origin)
        self.map.deleteConnection(cell[0], cell[1], neighbor)
        self._path = self._calculate_path()

    def _angle_degrees_to_neighbor(self, angle_degrees: float) -> int:
        if abs(angle_degrees) < 45:
            return 2
        if abs(angle_degrees) > 135:
            return 6
        if angle_degrees > 0:
            return 0
        return 4
