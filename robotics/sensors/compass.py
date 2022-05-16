import math
import time

from robotics.actuators import brickpi3


class Compass:
    _I2C_ADR = 0b00000010

    def __init__(self, BP: brickpi3.BrickPi3, port):
        self.port = port
        self.BP = BP
        BP.set_sensor_type(port, BP.SENSOR_TYPE.I2C, [0, 20])
        self._start_calibration()
        time.sleep(2)
        self._end_calibration()

    def get_angle(self) -> int:
        """
        Returns the direction of the compass in degrees, from -180 to 180.
        """
        get_dir = 0x42
        self.BP.transact_i2c(self.port, self._I2C_ADR, [get_dir], 2)
        value = self.BP.get_sensor(self.port)
        angle_degrees = ((value[0] & 0xFF) << 1) + value[1]

        if 0 <= angle_degrees <= 180:
            return math.radians(-angle_degrees)
        if 180 < angle_degrees <= 360:
            return math.radians(360 - angle_degrees)

    def _start_calibration(self):
        command = 0x41
        begin_calibration = 0x43
        self.BP.transact_i2c(self.port, self._I2C_ADR, [command, begin_calibration], 0)

    def _end_calibration(self):
        command = 0x41
        end_calibration = 0x00
        self.BP.transact_i2c(self.port, self._I2C_ADR, [command, end_calibration], 0)
