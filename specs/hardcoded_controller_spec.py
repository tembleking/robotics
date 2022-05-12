from hamcrest import is_, assert_that
from mamba import description, it, context

from robotics.geometry import Point, Location
from robotics.robot.controller import HardcodedController

with description('HardcodedController'):
    with context('when it starts from the white color'):
        with it('does the S first to the right'):
            controller = HardcodedController()

            # It starts from the colored square
            speed = controller.get_speed(Location.from_angle_degrees(Point(0.6, 2.8), -90))
            # Therefore it should get to the middle of the square
            assert_that(speed, is_((0.1, 0)))

            # It arrives the middle of the square
            speed = controller.get_speed(Location.from_angle_degrees(Point(0.6, 2.6), -90))
            # Therefore it should start turning to the right
            assert_that(speed, is_((0.0, -0.5)))

            # Finishes turning to the right
            speed = controller.get_speed(Location.from_angle_degrees(Point(0.6, 2.6), 180))
            # Therefore it should start the arc to the left
            assert_that(speed, is_((0.1, 0.25)))

            # Finishes the arc to the left
            speed = controller.get_speed(Location.from_angle_degrees(Point(0.6, 1.8), 0))
            # Therefore it should start the arc to the right
            assert_that(speed, is_((0.1, -0.25)))

            # Finishes the arc to the right
            speed = controller.get_speed(Location.from_angle_degrees(Point(0.6, 1.0), 180))
            # Therefore it should stop
            assert_that(speed, is_(None))

    with context('when it starts from the black color'):
        with it('does the S first to the left'):
            controller = HardcodedController()

            # It starts from the colored square
            speed = controller.get_speed(Location.from_angle_degrees(Point(2.2, 2.8), -90))
            # Therefore it should get to the middle of the square
            assert_that(speed, is_((0.1, 0)))

            # It arrives the middle of the square
            speed = controller.get_speed(Location.from_angle_degrees(Point(2.2, 2.6), -90))
            # Therefore it should start turning to the left
            assert_that(speed, is_((0.0, 0.5)))

            # Finishes turning to the left
            speed = controller.get_speed(Location.from_angle_degrees(Point(2.2, 2.6), 0))
            # Therefore it should start the arc to the right
            assert_that(speed, is_((0.1, -0.25)))

            # Finishes the arc to the right
            speed = controller.get_speed(Location.from_angle_degrees(Point(2.2, 1.8), 180))
            # Therefore it should start the arc to the left
            assert_that(speed, is_((0.1, 0.25)))

            # Finishes the arc to the left
            speed = controller.get_speed(Location.from_angle_degrees(Point(2.2, 1.0), 0))
            # Therefore it should stop
            assert_that(speed, is_(None))
