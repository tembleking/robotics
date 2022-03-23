import copy
import datetime
import math

import numpy


class _Coordinates:
    def __init__(self, coords):
        if isinstance(coords, list):
            self._matrix_coords = numpy.matrix(coords).transpose()
        elif isinstance(coords, numpy.matrix):
            self._matrix_coords = coords
        else:
            raise ValueError('coordinates are not either a list or a numpy.matrix')

    @property
    def x(self):
        return self._matrix_coords.item(0)

    @property
    def y(self):
        return self._matrix_coords.item(1)

    def rotate_degrees(self, degrees):
        return self.rotate_radians(math.radians(degrees))

    def rotate_radians(self, radians):
        transformation = numpy.matrix([[numpy.cos(radians), -numpy.sin(radians), 0],
                                       [numpy.sin(radians), numpy.cos(radians), 0],
                                       [0.0, 0.0, 1]])
        return _Coordinates(transformation * self._matrix_coords)

    def seen_from_other_location(self, other: 'Location'):
        transformation = numpy.matrix([[other.x_axis.x, other.y_axis.x, other.origin.x],
                                       [other.x_axis.y, other.y_axis.y, other.origin.y],
                                       [0.0, 0.0, 1.0]])
        return _Coordinates(transformation * self._matrix_coords)

    def __eq__(self, o: object) -> bool:
        return isinstance(o, _Coordinates) and numpy.allclose(self._matrix_coords, o._matrix_coords)

    def __repr__(self) -> str:
        return 'Coordinates(x=%s, y=%s)' % (self.x, self.y)


class Point(_Coordinates):
    def __init__(self, x, y):
        super().__init__([x, y, 1])

    def displace(self, x, y):
        return Point(self.x + x, self.y + y)

    def distance_to(self, other: 'Point'):
        return Direction(other.x - self.x, other.y - self.y).modulus()

    def __repr__(self) -> str:
        return 'Point(x=%s, y=%s)' % (self.x, self.y)

    @staticmethod
    def _from_coordinates(coords: '_Coordinates') -> 'Point':
        return Point(coords.x, coords.y)


class Direction(_Coordinates):
    def __init__(self, x, y):
        super().__init__([x, y, 0])

    def normalize(self):
        return Direction(self.x / self.modulus(), self.y / self.modulus())

    def modulus(self):
        return numpy.sqrt(self.x ** 2 + self.y ** 2)

    @staticmethod
    def _from_coordinates(coords: '_Coordinates') -> 'Direction':
        return Direction(coords.x, coords.y)


class Location:
    def __init__(self, origin: 'Point', x_axis: 'Direction', y_axis: 'Direction'):
        x_axis = Direction._from_coordinates(x_axis).normalize()
        y_axis = Direction._from_coordinates(y_axis).normalize()

        self._matrix_coords = numpy.matrix([
            [x_axis.x, y_axis.x, origin.x],
            [x_axis.y, y_axis.y, origin.y],
            [0, 0, 1],
        ])

    @staticmethod
    def from_angle_degrees(origin: 'Point', angle: float):
        return Location.from_angle_radians(origin, math.radians(angle))

    @staticmethod
    def from_angle_radians(origin: 'Point', angle: float):
        return Location(origin, Direction(1, 0).rotate_radians(angle), Direction(0, 1).rotate_radians(angle))

    @property
    def x_axis(self) -> 'Direction':
        return Direction(self._matrix_coords.item((0, 0)), self._matrix_coords.item((1, 0))).normalize()

    @property
    def y_axis(self) -> 'Direction':
        return Direction(self._matrix_coords.item((0, 1)), self._matrix_coords.item((1, 1))).normalize()

    @property
    def origin(self) -> 'Point':
        return Point(self._matrix_coords.item((0, 2)), self._matrix_coords.item((1, 2)))

    def angle_radians(self):
        return math.atan2(self.x_axis.y, self.x_axis.x)

    def angle_degrees(self):
        return math.degrees(self.angle_radians())

    def inverse(self):
        location = copy.deepcopy(self)
        location._matrix_coords = numpy.linalg.inv(location._matrix_coords)
        return location

    def seen_from_other_location(self, other: 'Location'):
        location = copy.deepcopy(self)
        transformation = numpy.matrix([[other.x_axis.x, other.y_axis.x, other.origin.x],
                                       [other.x_axis.y, other.y_axis.y, other.origin.y],
                                       [0.0, 0.0, 1.0]])
        location._matrix_coords = transformation.dot(location._matrix_coords)
        return location

    def radius_of_curvature(self):
        if self.origin.y == 0:
            return math.inf
        return (self.origin.x ** 2 + self.origin.y ** 2) / (2.0 * self.origin.y)

    def angle_of_curvature(self):
        return math.atan2(self.origin.x * self.origin.y * 2, self.origin.x ** 2 - self.origin.y ** 2)

    def angular_velocity_to_arrive_in(self, duration: datetime.timedelta):
        return self.angle_of_curvature() / duration.seconds

    def linear_velocity_to_arrive_in(self, duration: datetime.timedelta):
        return self.angular_velocity_to_arrive_in(duration) * self.radius_of_curvature()

    def new_location_after_rotating(self, movement_ratio=1) -> 'Location':
        angle_of_curvature = self.angle_of_curvature() * movement_ratio
        new_x_position = self.radius_of_curvature() * math.sin(angle_of_curvature)
        new_y_position = self.radius_of_curvature() * (1 - math.cos(angle_of_curvature))
        return Location.from_angle_radians(Point(new_x_position, new_y_position), angle_of_curvature)

    def __eq__(self, o: object) -> bool:
        return isinstance(o, Location) and numpy.allclose(self._matrix_coords, o._matrix_coords)

    def __repr__(self) -> str:
        return 'Location(origin=%s, x_axis=%s, y_axis=%s' % (self.origin, self.x_axis, self.y_axis)


class PolarCoordinates:
    def __init__(self, location: 'Location'):
        self._rho = math.sqrt(location.origin.x ** 2 + location.origin.y ** 2)
        self._beta = PolarCoordinates._normalize_radians(math.atan2(location.origin.x, location.origin.y) + math.pi)
        self._alpha = self._beta - location.angle_radians()

    @property
    def rho(self):
        return self._rho

    @property
    def beta(self):
        return self._beta

    @property
    def alpha(self):
        return self._alpha

    def _normalize_radians(angle: float):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def __repr__(self) -> str:
        return f'PolarCoordinates(rho={self.rho}, beta={self.beta}, alpha={self.alpha})'