from unittest.mock import MagicMock, call
import math

from hamcrest import assert_that, is_
from mamba import description, it
from robotics.geometry import Location, Point
from robotics.robot.controller import Controller
from robotics.robot.trajectory_generator import TrajectoryGenerator

with description('controller', 'unit') as self:
    with it('sends the robot to the next location in a straight line'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(0.2, 0), 0),
            Location.from_angle_degrees(Point(0.40, 0), 0),
            Location.from_angle_degrees(Point(0.60, 0), 0),
            Location.from_angle_degrees(Point(0.80, 0), 0),
            Location.from_angle_degrees(Point(1, 0), 0),
            Location.from_angle_degrees(Point(1.2, 0), 0),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()
        trajectory_generator = TrajectoryGenerator([
            Location.from_angle_degrees(Point(0.40, 0), 0),
            Location.from_angle_degrees(Point(1.20, 0), 0),
        ])
        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator,
                                     k_rho=1,
                                     k_alpha=1.2,
                                     k_beta=1)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.2, 0), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(6))

    with it('sends the robot to the next location in a straight line'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 90),
            Location.from_angle_degrees(Point(0, .2), 90),
            Location.from_angle_degrees(Point(0, .4), 90),
            Location.from_angle_degrees(Point(0, .6), 90),
            Location.from_angle_degrees(Point(0, .8), 90),
            Location.from_angle_degrees(Point(0, 1), 90),
            Location.from_angle_degrees(Point(0, 1.2), 90),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()
        trajectory_generator = TrajectoryGenerator([
            Location.from_angle_degrees(Point(0, 0.4), 90),
            Location.from_angle_degrees(Point(0, 1.2), 90),
        ])

        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator,
                                     k_rho=1,
                                     k_alpha=1.2,
                                     k_beta=1)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.2, 0), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(6))

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

        trajectory_generator = TrajectoryGenerator([Location.from_angle_degrees(Point(0, 0), 90)])
        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator,
                                     k_rho=0.5,
                                     k_alpha=0.6,
                                     k_beta=1)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.0, -4.084), call(0.0, -4.398), call(0.0, -4.712), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(4))

    with it('describes an arc to arrive to the destination'):
        odometry = MagicMock()
        odometry.location.side_effect = [
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(math.sqrt(2) * 0.40 / 2, math.sqrt(2) * 0.40 / 2), 45),
            Location.from_angle_degrees(Point(.40, .40), 90),
        ]
        robot = MagicMock()
        robot.set_speed = MagicMock()

        trajectory_generator = TrajectoryGenerator([Location.from_angle_degrees(Point(0.40, 0.40), 90)])
        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator,
                                     k_rho=0.5,
                                     k_alpha=0.6,
                                     k_beta=0.6)
        self.controller.start()

        robot.set_speed.assert_has_calls([call(0.283, -0.0), call(0.083, -0.471), call(0, 0)])
        assert_that(robot.set_speed.call_count, is_(3))

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
        trajectory_generator = TrajectoryGenerator([Location.from_angle_degrees(Point(120, 0), 0)])

        self.controller = Controller(odometry=odometry, robot=robot, polling_period=0.2,
                                     trajectory_generator=trajectory_generator,
                                     k_rho=0.5,
                                     k_alpha=0.6,
                                     k_beta=1)
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
