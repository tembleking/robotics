import math
import time
from unittest.mock import MagicMock

from mamba import description, context, it
from hamcrest import assert_that, is_, close_to

from robotics.robot.odometry import Odometry

with description('Odometry', 'unit') as self:
    with context('when it receives the angles from the wheels'):
        with it('tells us the odometry speed when going straight'):
            left_motor = MagicMock()
            left_motor.get_last_angle.return_value = math.radians(5)
            right_motor = MagicMock()
            right_motor.get_last_angle.return_value = math.radians(5)

            odometry = Odometry(left_motor, right_motor, polling_period=0.01,
                                wheel_radius=0.0265, axis_length=0.119)

            v, w = odometry.read_speed()
            assert_that(v, is_(close_to(0.2312, 0.0001)))
            assert_that(w, is_(0))

        with it('tells us the odometry speed for a turn to the right'):
            left_motor = MagicMock()
            left_motor.get_last_angle.return_value = math.radians(5)
            right_motor = MagicMock()
            right_motor.get_last_angle.return_value = math.radians(-5)

            odometry = Odometry(left_motor, right_motor, polling_period=0.01,
                                wheel_radius=0.0265, axis_length=0.119)

            v, w = odometry.read_speed()
            assert_that(v, is_(0))
            assert_that(w, is_(close_to(-3.8866, 0.0001)))

        with it('tells us the odometry speed for a turn to the left'):
            left_motor = MagicMock()
            left_motor.get_last_angle.return_value = math.radians(-5)
            right_motor = MagicMock()
            right_motor.get_last_angle.return_value = math.radians(5)

            odometry = Odometry(left_motor, right_motor, polling_period=0.01,
                                wheel_radius=0.0265, axis_length=0.119)

            v, w = odometry.read_speed()
            assert_that(v, is_(0))
            assert_that(w, is_(close_to(3.8866, 0.0001)))

        with it('tells us the odometry speed for an arc'):
            left_motor = MagicMock()
            left_motor.get_last_angle.return_value = math.radians(2)
            right_motor = MagicMock()
            right_motor.get_last_angle.return_value = math.radians(5)

            odometry = Odometry(left_motor, right_motor, polling_period=0.01,
                                wheel_radius=0.0265, axis_length=0.119)

            v, w = odometry.read_speed()
            assert_that(v, is_(close_to(0.1618, 0.0001)))
            assert_that(w, is_(close_to(1.1659, 0.0001)))

    with context('when calculating the position after a certain period of time'):
        with it('tells us the odometry position when going straight'):
            left_motor = MagicMock()
            left_motor.get_last_angle.return_value = math.radians(5)
            right_motor = MagicMock()
            right_motor.get_last_angle.return_value = math.radians(5)

            odometry = Odometry(left_motor, right_motor, polling_period=0.01,
                                wheel_radius=0.0265, axis_length=0.119)
            odometry.start()

            time.sleep(1)
            odometry.stop()

            location = odometry.location()
            assert_that(location.origin.x, is_(close_to(0.2312, 0.01)))
            assert_that(location.origin.y, is_(0))
            assert_that(location.angle_radians(), is_(0))

        with it('tells us the odometry position when it spins'):
            left_motor = MagicMock()
            left_motor.get_last_angle.return_value = math.radians(5)
            right_motor = MagicMock()
            right_motor.get_last_angle.return_value = math.radians(-5)

            odometry = Odometry(left_motor, right_motor, polling_period=0.01,
                                wheel_radius=0.0265, axis_length=0.119)
            odometry.start()

            time.sleep(1)
            odometry.stop()
            location = odometry.location()
            assert_that(location.origin.x, is_(close_to(0, 0.00001)))
            assert_that(location.origin.y, is_(close_to(0, 0.00001)))
            assert_that(location.angle_radians(), is_(close_to(2.3965, 0.001)))
