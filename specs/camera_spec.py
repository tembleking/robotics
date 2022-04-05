from unittest.mock import MagicMock, call
import math

from hamcrest import assert_that, is_
from mamba import description, it
from robotics.geometry import Location, Point
from robotics.sensors.camera import Camera

with description('camera', 'unit') as self:
    with fit('detects the blob in the image correctly'):
        camera = Camera()
        blob_point, blob_area = camera.detect_blob()
        assert_that(blob_point, is_())