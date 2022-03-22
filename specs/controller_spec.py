from unittest.mock import MagicMock
import math

from expects import expect, equal
from mamba import description, it
from robotics.geometry import Location, Point
from robotics.robot.controller import Controller


class FakeTrajectoryGenerator:
    def __init__(self, points_to_visit):
        self.points_to_visit = points_to_visit

    def next_absolute_point_to_visit(self) -> Point:
        try:
            return self.points_to_visit[0]
        except IndexError:
            return None

    def mark_point_as_visited(self):
        self.points_to_visit = self.points_to_visit[1:]


with description('controller', 'unit') as self:
    with it('sends the robot to the next location in a straight line'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(20, 0), 0),
            Location.from_angle_degrees(Point(40, 0), 0),
            Location.from_angle_degrees(Point(60, 0), 0),
            Location.from_angle_degrees(Point(80, 0), 0),
            Location.from_angle_degrees(Point(100, 0), 0),
            Location.from_angle_degrees(Point(120, 0), 0),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()
        trajectory_generator = FakeTrajectoryGenerator([
            Location.from_angle_degrees(Point(40, 0), 0),
            Location.from_angle_degrees(Point(120, 0), 0),
        ])
        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        robot.set_speed.assert_called_with(15, 0)
        expect(robot.set_speed.call_count).to(equal(5))

    with it('turns around'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(0, 0), 30),
            Location.from_angle_degrees(Point(0, 0), 60),
            Location.from_angle_degrees(Point(0, 0), 90),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()

        trajectory_generator = FakeTrajectoryGenerator([Location.from_angle_degrees(Point(0, 0), 90)])
        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        robot.set_speed.assert_called_with(0, 1.571)
        expect(robot.set_speed.call_count).to(equal(3))

    with it('describes an arc to arrive to the destination'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(40, 0), 90),
            Location.from_angle_degrees(Point(math.sqrt(2) * 40 / 2, math.sqrt(2) * 40 / 2), 135),
            Location.from_angle_degrees(Point(0, 40), 180),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()

        trajectory_generator = FakeTrajectoryGenerator([Location.from_angle_degrees(Point(0, 40), 180)])
        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        robot.set_speed.assert_called_with(15, 15 / 40)
        expect(robot.set_speed.call_count).to(equal(2))

    with it('stores the locations of the visited points'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(20, 0), 0),
            Location.from_angle_degrees(Point(40, 0), 0),
            Location.from_angle_degrees(Point(60, 0), 0),
            Location.from_angle_degrees(Point(80, 0), 0),
            Location.from_angle_degrees(Point(100, 0), 0),
            Location.from_angle_degrees(Point(120, 0), 0),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()
        trajectory_generator = FakeTrajectoryGenerator([Location.from_angle_degrees(Point(120, 0), 0)])

        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()
        expect(self.controller.visited_points).to(equal([
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(20, 0), 0),
            Location.from_angle_degrees(Point(40, 0), 0),
            Location.from_angle_degrees(Point(60, 0), 0),
            Location.from_angle_degrees(Point(80, 0), 0),
            Location.from_angle_degrees(Point(100, 0), 0),
            Location.from_angle_degrees(Point(120, 0), 0),
        ]))
