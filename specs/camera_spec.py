from typing import Tuple
from unittest.mock import MagicMock

import cv2
import numpy as np
from hamcrest import assert_that, is_, close_to, none
from mamba import description, it, before

from robotics.sensors.camera import Camera


def load_image(image_name: str) -> Tuple[bool, np.ndarray]:
    return True, cv2.imread('specs/fixtures/ball_images/%s' % image_name, cv2.IMREAD_COLOR)


with description('camera', 'unit') as self:
    with before.each:
        self.camera_sensor = Camera(video_capturer=MagicMock())

    with it('detects the blob in the centered image correctly'):
        self.camera_sensor.video_capturer.read.return_value = load_image('centered_ball.png')

        blob_point, blob_area = self.camera_sensor.get_blob_position_and_size()

        assert_that(blob_point.x, is_(close_to(383.550598, 0.0001)))
        assert_that(blob_point.y, is_(close_to(272.60113, 0.0001)))
        assert_that(blob_area, is_(close_to(259.83825, 0.0001)))

    with it('detects the blob when the ball is far away'):
        self.camera_sensor.video_capturer.read.return_value = load_image('ball_far_away.png')

        blob_point, blob_area = self.camera_sensor.get_blob_position_and_size()

        assert_that(blob_point.x, is_(close_to(287.369079, 0.0001)))
        assert_that(blob_point.y, is_(close_to(84.53061, 0.0001)))
        assert_that(blob_area, is_(close_to(97.62372, 0.0001)))

    with it('detects the blob when the robot is about to reach it and stop'):
        self.camera_sensor.video_capturer.read.return_value = load_image('about_to_stop.png')

        blob_point, blob_area = self.camera_sensor.get_blob_position_and_size()

        assert_that(blob_point.x, is_(close_to(313.66549, 0.0001)))
        assert_that(blob_point.y, is_(close_to(340.60589, 0.0001)))
        assert_that(blob_area, is_(close_to(250.916961, 0.0001)))

    with it("doesn't detect the blob if the ball is in the claws"):
        self.camera_sensor.video_capturer.read.return_value = load_image('ball_in_claws.png')

        blob_point, blob_area = self.camera_sensor.get_blob_position_and_size()

        assert_that(blob_point, is_(none()))
        assert_that(blob_area, is_(none()))

    with it("doesn't detect the blob if the ball is not found"):
        self.camera_sensor.video_capturer.read.return_value = load_image('ball_not_found.png')

        blob_point, blob_area = self.camera_sensor.get_blob_position_and_size()

        assert_that(blob_point, is_(none()))
        assert_that(blob_area, is_(none()))
