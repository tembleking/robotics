import math

from robotics.actuators import brickpi3


class Gyro:
    def __init__(self, BP: brickpi3.BrickPi3, port, offset=0):
        self.BP = BP
        self.port = port
        self.offset = offset
        BP.set_sensor_type(port, BP.SENSOR_TYPE.EV3_GYRO_ABS)
        self.wait_until_sensor_is_initialized()
        print('Gyro initialized')

    def wait_until_sensor_is_initialized(self):
        while True:
            try:
                self.BP.get_sensor(self.port)
                break
            except brickpi3.SensorError:
                pass

    def get_angle(self) -> float:
        """
        Retrieves the angle normalized in radians from -math.pi to math.pi
        :return:
        """

        degree_angles = self.BP.get_sensor(self.port) - math.degrees(self.offset)
        normalized_degree_angles = -(degree_angles % 360)
        if normalized_degree_angles > 180:
            normalized_degree_angles -= 360
        return math.radians(normalized_degree_angles)
