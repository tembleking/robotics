from mamba import description, it, fit
from hamcrest import assert_that, is_, close_to
from unittest.mock import MagicMock
from robotics.geometry import Point
from robotics.robot.ball_following_speed_generator import BallFollowingSpeedGenerator
with description('BallFollowingTrajectoryGenerator', 'unit') as self:
    with it('should rotate to the left to find the ball'):
        ball_following_trajectory_generator = BallFollowingSpeedGenerator()
        v, w = ball_following_trajectory_generator.next_speed()
        assert_that(v, is_(close_to(0.0, 0)))
        assert_that(w, is_(close_to(20.0, 0)))

    with fit('should get closer to the ball when it is in front of the robot'):
        camera = MagicMock()
        ball_following_trajectory_generator = BallFollowingSpeedGenerator(
            camera=camera,
            area_goal=259,
            distance_goal=313,
            distance_damping=1,
            area_damping=1,
        )
        camera.get_blob_position_and_size.return_value = (Point(313, 0), 200)
        v, w = ball_following_trajectory_generator.next_speed()
        assert_that(v, is_(close_to(59.0, 0.1)))
        assert_that(w, is_(close_to(0, 0.01)))
