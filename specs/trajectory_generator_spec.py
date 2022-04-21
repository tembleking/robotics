from unittest.mock import MagicMock

from hamcrest import assert_that, is_, close_to
from mamba import fdescription, it

from robotics.geometry import Location, Point
from robotics.robot.trajectory_generator import TrajectoryGenerator

with fdescription('Trajectory Generator', 'unit') as self:
    with it('returns the next points until they are marked as visited'):
        map = MagicMock()
        map.sizeCell.return_value = 400
        robot = MagicMock()
        robot.location.return_value = Location.from_angle_degrees(Point(0.2, 0.2), 0)
        map.findPath.return_value = [(0, 0), (1, 1), (2, 2)]

        generator = TrajectoryGenerator(robot=robot, map=map, destination=Point(2, 2))

        point_to_visit = generator.next_absolute_point_to_visit()
        assert_that(point_to_visit.origin.x, is_(close_to(0.6, 0.001)))
        assert_that(point_to_visit.origin.y, is_(close_to(0.6, 0.001)))
        assert_that(point_to_visit.angle_degrees(), is_(close_to(45, 0.001)))
        map.findPath.assert_called_with((0, 0), (2, 2))

    with it('returns the next points until they are marked as visited'):
        map = MagicMock()
        map.sizeCell.return_value = 400
        robot = MagicMock()
        robot.location.return_value = Location.from_angle_degrees(Point(0.2, 0.6), 0)
        map.findPath.return_value = [(0, 1), (1, 1), (2, 2)]

        generator = TrajectoryGenerator(robot=robot, map=map, destination=Point(2, 2))

        point_to_visit = generator.next_absolute_point_to_visit()
        assert_that(point_to_visit.origin.x, is_(close_to(0.6, 0.001)))
        assert_that(point_to_visit.origin.y, is_(close_to(0.6, 0.001)))
        assert_that(point_to_visit.angle_degrees(), is_(close_to(0, 0.001)))
        map.findPath.assert_called_with((0, 1), (2, 2))
