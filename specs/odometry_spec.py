import time
import math
from unittest.mock import MagicMock

from hamcrest import assert_that, is_, close_to

from robotics.geometry import Point, Direction, Location

with description('Odometry', 'unit') as self:
    with context ('when it receives the angles from the wheels'):
    	with it('tells us the odometry speed'):
    		left_motor = MagicMock()
    		left_motor.get_last_angle.return_value = math.radians(5)
    		right_motor = MagicMock()
    		right_motor.get_last_angle.return_value = math.radians(5)
    		odometry = Odometry(left_motor, right_motor, polling_period = 0.01)
    		odometry.start()

    		time.sleep(1)
	        v, w = odometry.read_speed()
	        assert_that(v, is_(close_to(46.22, 0.01)))
	        assert_that(w, is_(0))