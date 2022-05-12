from unittest.mock import MagicMock

from hamcrest import assert_that, is_
from mamba import description, it, before

from robotics.geometry import Location, Point
from robotics.robot.obstacle_trajectory_speed_generator import ObstacleTrajectorySpeedGenerator


class FakeTrajectoryGenerator:
    def __init__(self, points_to_visit=None):
        if points_to_visit is None:
            self.points_to_visit = []
        else:
            self.points_to_visit = points_to_visit
        self.mark_wall_ahead_called = False

    def set_points_to_vist(self, points_to_visit: list):
        self.points_to_visit = points_to_visit

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


with description('ObstacleTrajectorySpeedGenerator', 'unit') as self:
    with before.each:
        self.trajectory_generator = FakeTrajectoryGenerator()
        self.obstacle_detector = MagicMock()
        self.obstacle_detector.obstacle_detected.return_value = False
        self.controller = ObstacleTrajectorySpeedGenerator(trajectory_generator=self.trajectory_generator,
                                                           obstacle_detector=self.obstacle_detector)

    with it('sends the robot to the next location in a straight line'):
        self.trajectory_generator.set_points_to_vist([
            Location.from_angle_degrees(Point(0.40, 0), 0),
            Location.from_angle_degrees(Point(1.20, 0), 0)]
        )

        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0, 0), 0)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0, 0), 0)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0.40, 0), 0)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0.4000001, 0), 0)), is_((0.0, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.20, 0), 0)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.2000001, 0), 0)), is_((0.0, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.2000001, 0), 0)), is_(None))

    with it('turns around to the left'):
        self.trajectory_generator.set_points_to_vist([Location.from_angle_degrees(Point(0, 0), 90)])

        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0, 0), 0)), is_((0, 0.25)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0, 0), 90)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0, 0.00001), 90)), is_((0, 0)))

    with it('marks a new position as well when it encounters a new obstacle'):
        self.obstacle_detector.obstacle_detected.return_value = True
        self.trajectory_generator.set_points_to_vist([
            Location.from_angle_degrees(Point(0.4, 0), 0),
            Location.from_angle_degrees(Point(0, 0.4), 90),
        ])
        self.controller.get_speed(Location.from_angle_degrees(Point(0, 0), 0))

        assert_that(self.trajectory_generator.mark_wall_ahead_called, is_(True))

    with it('does an L'):
        self.trajectory_generator.set_points_to_vist([
            Location.from_angle_degrees(Point(1, 0), 0),
            Location.from_angle_degrees(Point(1, 0), 90),
            Location.from_angle_degrees(Point(1, 1), 90),
        ])

        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(0, 0), 0)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1, 0), 0)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 0), 0)), is_((0, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 0), 0)), is_((0, 0.25)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 0), 90)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 0.00001), 90)), is_((0.0, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 1), 90)), is_((0.1, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 1.00001), 90)), is_((0, 0)))
        assert_that(self.controller.get_speed(Location.from_angle_degrees(Point(1.00001, 1.00001), 90)), is_(None))
