from unittest.mock import MagicMock

import cv2
import numpy as np
from hamcrest import assert_that, is_
from mamba import description, it

from robotics.sensors.camera import Camera


def generate_default_blob_detector_params() -> cv2.SimpleBlobDetector_Params:
    params = cv2.SimpleBlobDetector_Params()

    # Thresholds (no clue about the meaning of this values)
    params.minThreshold = 400
    params.maxThreshold = 4000

    # Filter by Area
    #   hard to define, but this works quite well
    #   even though we might have to lower the minArea
    params.filterByArea = True
    params.minArea = 1000
    params.maxArea = 50000

    # Filter by Circularity
    # The area has to define a circle
    params.filterByCircularity = True
    params.minCircularity = 0.01

    params.filterByConvexity = True
    params.minConvexity = 0.4
    params.maxConvexity = 0.9

    # Dont filter by color (we do it with hsv values), Convexity or Inertia
    params.filterByColor = False

    params.filterByInertia = False
    params.minInertiaRatio = 0.1
    params.maxInertiaRatio = 1
    return params


def load_centered_image() -> np.ndarray:
    return cv2.imread('fixtures/centered_image.png', cv2.IMREAD_COLOR)


with description('camera', 'unit') as self:
    with it('detects the blob in the image correctly'):
        camera_sensor = Camera(
            video_capturer=MagicMock(),
            min_light_settings=[0, 124, 0],
            min_dark_settings=[0, 85, 0],
            params=generate_default_blob_detector_params())
        camera_sensor.video_capturer.read.return_value = load_centered_image()
        blob_point, blob_area = camera_sensor.get_blob_position_and_size()
        assert_that(blob_point, is_())
        assert_that(blob_area, is_())
