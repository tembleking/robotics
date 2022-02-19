import math

import numpy


class Basis:
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
        return Basis(origin, Direction(1, 0).rotate_degrees(angle), Direction(0, 1).rotate_degrees(angle))

    @staticmethod
    def from_angle_radians(origin: 'Point', angle: float):
        return Basis(origin, Direction(1, 0).rotate_radians(angle), Direction(0, 1).rotate_radians(angle))

    def x_axis(self) -> 'Direction':
        return Direction(self._matrix_coords.item((0, 0)), self._matrix_coords.item((1, 0)))

    def y_axis(self) -> 'Direction':
        return Direction(self._matrix_coords.item((0, 1)), self._matrix_coords.item((1, 1)))

    def origin(self) -> 'Point':
        return Point(self._matrix_coords.item((0, 2)), self._matrix_coords.item((1, 2)))

    def angle_radians(self):
        return math.atan2(self.x_axis().y, self.x_axis().x)

    def angle_degrees(self):
        return self.angle_radians() * 180 / math.pi


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
        return self.rotate_radians(degrees * numpy.pi / 180.0)

    def rotate_radians(self, radians):
        transformation = numpy.matrix([[numpy.cos(radians), -numpy.sin(radians), 0],
                                       [numpy.sin(radians), numpy.cos(radians), 0],
                                       [0.0, 0.0, 1]])
        return _Coordinates(transformation * self._matrix_coords)

    def seen_from_other_basis(self, other: 'Basis'):
        transformation = numpy.matrix([[other.x_axis().x, other.y_axis().x, other.origin().x],
                                       [other.x_axis().y, other.y_axis().y, other.origin().y],
                                       [0.0, 0.0, 1.0]])
        return _Coordinates(transformation * self._matrix_coords)

    def __eq__(self, o: object) -> bool:
        return isinstance(o, _Coordinates) and numpy.allclose(self._matrix_coords, o._matrix_coords)

    def __repr__(self) -> str:
        return f'Coordinates(x={self.x}, y={self.y})'


class Point(_Coordinates):
    def __init__(self, x, y):
        super().__init__([x, y, 1])

    def displace(self, x, y):
        return Point(self.x + x, self.y + y)

    def __repr__(self) -> str:
        return f'Point(x={self.x}, y={self.y})'

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
