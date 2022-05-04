from robotics.actuators import brickpi3


class Light:
    def __init__(self, BP: brickpi3.BrickPi3, port):
        self.BP = BP
        self.port = port
        BP.set_sensor_type(port, BP.SENSOR_TYPE.NXT_LIGHT_ON)

    def is_white(self):
        return self.BP.get_sensor(self.port) < 1800
