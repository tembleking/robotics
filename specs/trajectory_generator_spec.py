from mamba import description, it
from hamcrest import assert_that, is_

from robotics.geometry import Point, Location
from robotics.robot.trajectory_generator import TrajectoryGenerator

with description('Trajectory Generator', 'unit') as self:
    with it('returns the next points until they are marked as visited'):
        generator = TrajectoryGenerator([
            Location.from_angle_degrees(Point(0, 0), 0),
            Location.from_angle_degrees(Point(1, 0), 0),
        ])

        assert_that(generator.next_absolute_point_to_visit(), is_(Location.from_angle_degrees(Point(0, 0), 0)))
        assert_that(generator.next_absolute_point_to_visit(), is_(Location.from_angle_degrees(Point(0, 0), 0)))

        generator.mark_point_as_visited()
        assert_that(generator.next_absolute_point_to_visit(), is_(Location.from_angle_degrees(Point(1, 0), 0)))
