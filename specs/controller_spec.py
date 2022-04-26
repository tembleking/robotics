from unittest.mock import MagicMock, call
import math

from hamcrest import assert_that, is_
from mamba import description, it, _it
from robotics.geometry import Location, Point
from robotics.robot.controller import Controller


class FakeTrajectoryGenerator:
    def __init__(self, points_to_visit):
        self.points_to_visit = points_to_visit
        self.mark_wall_ahead_called = False

    def next_absolute_point_to_visit(self) -> Point:
        try:
            return self.points_to_visit[0]
        except IndexError:
            return None

    def mark_point_as_visited(self):
        self.points_to_visit = self.points_to_visit[1:]

    def mark_wall_ahead(self):
        self.points_to_visit = self.points_to_visit[1:]
        self.mark_wall_ahead_called = True


with description('controller', 'unit') as self:
    with it('sends the robot to the next location in a straight line'):
        (MagicMock()).set_speed = MagicMock()
        MagicMock().location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(0.2, 0), 0),
            Location.from_angle_degrees(Point(0.40, 0), 0),
            Location.from_angle_degrees(Point(0.41, 0), 0),
            Location.from_angle_degrees(Point(0.80, 0), 0),
            Location.from_angle_degrees(Point(1, 0), 0),
            Location.from_angle_degrees(Point(1.2, 0), 0),
            Location.from_angle_degrees(Point(1.21, 0), 0),
        ]
        trajectory_generator = FakeTrajectoryGenerator([
            Location.from_angle_degrees(Point(0.40, 0), 0),
            Location.from_angle_degrees(Point(1.20, 0), 0),
        ])
        self.controller = Controller(robot=(MagicMock()), polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        MagicMock().set_speed.assert_has_calls([call(0.1, 0), call(0, 0)])
        assert_that(MagicMock().set_speed.call_count, is_(7))

    with it('sends the robot to the next location in a straight line'):
        robot = MagicMock()
        robot.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 90),
            Location.from_angle_degrees(Point(0, .2), 90),
            Location.from_angle_degrees(Point(0, .4), 90),
            Location.from_angle_degrees(Point(0, .6), 90),
            Location.from_angle_degrees(Point(0, .8), 90),
            Location.from_angle_degrees(Point(0, 1), 90),
            Location.from_angle_degrees(Point(0, 1.2), 90),
        ]
        robot.set_speed = MagicMock()
        trajectory_generator = FakeTrajectoryGenerator([
            Location.from_angle_degrees(Point(0, 0.4), 90),
            Location.from_angle_degrees(Point(0, 1.2), 90),
        ])

        self.controller = Controller(robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.2, 0), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(6))

    with it('turns around to the left'):
        robot = MagicMock()
        robot.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(0, 0), 30),
            Location.from_angle_degrees(Point(0, 0), 60),
            Location.from_angle_degrees(Point(0, 0), 90),
        ]
        robot.set_speed = MagicMock()

        trajectory_generator = FakeTrajectoryGenerator([Location.from_angle_degrees(Point(0, 0), 90)])
        self.controller = Controller(robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.0, 0.5), call(0.0, 0.5), call(0.0, 0.5), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(4))

    with _it('describes an arc to arrive to the destination'):
        # FIXME(fede): The current implementation does not describe arcs.
        robot = MagicMock()
        robot.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(math.sqrt(2) * 0.40 / 2, math.sqrt(2) * 0.40 / 2), 45),
            Location.from_angle_degrees(Point(.40, .40), 90),
        ]
        robot.set_speed = MagicMock()

        trajectory_generator = FakeTrajectoryGenerator([Location.from_angle_degrees(Point(0.40, 0.40), 90)])
        self.controller = Controller(robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.283, -0.0), call(0.083, -0.471), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(3))

    with it('stores the locations of the visited points'):
        robot = MagicMock()
        robot.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(20, 0), 0),
            Location.from_angle_degrees(Point(40, 0), 0),
            Location.from_angle_degrees(Point(60, 0), 0),
            Location.from_angle_degrees(Point(80, 0), 0),
            Location.from_angle_degrees(Point(100, 0), 0),
            Location.from_angle_degrees(Point(120, 0), 0),
        ]
        robot.set_speed = MagicMock()
        trajectory_generator = FakeTrajectoryGenerator([Location.from_angle_degrees(Point(120, 0), 0)])

        self.controller = Controller(robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator)
        self.controller.start()
        assert_that(self.controller.visited_points, is_([
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(20, 0), 0),
            Location.from_angle_degrees(Point(40, 0), 0),
            Location.from_angle_degrees(Point(60, 0), 0),
            Location.from_angle_degrees(Point(80, 0), 0),
            Location.from_angle_degrees(Point(100, 0), 0),
            Location.from_angle_degrees(Point(120, 0), 0),
        ]))

    with it('marks a new position as well when it encounters a new obstacle'):
        obstacle_detector = MagicMock()
        obstacle_detector.obstacle_detected.return_value = True
        trajectory_generator = FakeTrajectoryGenerator([
            Location.from_angle_degrees(Point(0.4, 0), 0),
            Location.from_angle_degrees(Point(0, 0.4), 90),
            ])
        robot = MagicMock()
        robot.location.side_effect = [Location.from_angle_degrees(Point(0, 0), 0),
                                      Location.from_angle_degrees(Point(120, 0), 0),
                                      Location.from_angle_degrees(Point(121, 0), 0)]
        controller = Controller(robot=robot, polling_period=0.2, trajectory_generator=trajectory_generator,
                                obstacle_detector=obstacle_detector)
        controller.start()
        assert_that(trajectory_generator.mark_wall_ahead_called, is_(True))
