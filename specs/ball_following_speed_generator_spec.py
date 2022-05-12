from mamba import description, it
from hamcrest import assert_that, is_, close_to
from unittest.mock import MagicMock
from robotics.geometry import Point
from robotics.robot.ball_following_speed_generator import BallFollowingSpeedGenerator

with description('BallFollowingSpeedGenerator', 'unit') as self:
    with it('should rotate to the left to find the ball'):
        camera = MagicMock()
        ball_following_trajectory_generator = BallFollowingSpeedGenerator(
            camera=camera,
            robot=MagicMock(),
            area_goal=214,
            distance_goal=267,
            distance_damping=0.001,
            area_damping=0.001,
        )
        camera.get_blob_position_and_size.return_value = (None, None)
        v, w = ball_following_trajectory_generator.get_speed(None)
        assert_that(v, is_(close_to(0.0, 0)))
        assert_that(abs(w), is_(close_to(0.5, 0)))

    with it('should get closer to the ball when it is in front of the robot'):
        camera = MagicMock()
        ball_following_trajectory_generator = BallFollowingSpeedGenerator(
            camera=camera,
            robot=MagicMock(),
            area_goal=214,
            distance_goal=267,
            distance_damping=0.001,
            area_damping=0.001,
        )
        camera.get_blob_position_and_size.return_value = (Point(313, 0), 200)
        v, w = ball_following_trajectory_generator.get_speed(None)
        assert_that(v, is_(close_to(0.014, 0.1)))
        assert_that(w, is_(close_to(-0.04, 0.01)))
