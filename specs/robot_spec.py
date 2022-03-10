from mamba import describe, context, it, before
from doublex import Spy, assert_that, called
from robotics.robot.robot import Robot

with describe('robot', 'unit') as self:
    with before.each:
        self.left_motor = Spy()
        self.right_motor = Spy()
        self.robot = Robot(
            left_motor=self.left_motor,
            right_motor=self.right_motor,
            claw_motor=None,
            wheel_radius=1,
            axis_length=1
        )

    with context('we tell the robot to go at a certain speed'):
        with it('stops the robot'):
            self.robot.set_speed(0, 0)

            assert_that(self.left_motor.set_speed, called().with_args(0))
            assert_that(self.right_motor.set_speed, called().with_args(0))

        with it('goes in a straight line'):
            self.robot.set_speed(1, 0)

            assert_that(self.left_motor.set_speed, called().with_args(1))
            assert_that(self.right_motor.set_speed, called().with_args(1))

        with it('turns left'):
            self.robot.set_speed(0, 1)

            assert_that(self.left_motor.set_speed, called().with_args(-0.5))
            assert_that(self.right_motor.set_speed, called().with_args(0.5))

        with it('turns right'):
            self.robot.set_speed(0, -1)

            assert_that(self.left_motor.set_speed, called().with_args(0.5))
            assert_that(self.right_motor.set_speed, called().with_args(-0.5))

        with it('turns while advancing'):
            self.robot.set_speed(1, 1)

            assert_that(self.left_motor.set_speed, called().with_args(0.5))
            assert_that(self.right_motor.set_speed, called().with_args(1.5))
