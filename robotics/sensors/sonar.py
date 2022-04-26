from robotics.actuators import brickpi3


class Sonar:
    def __init__(self, BP: brickpi3.BrickPi3, connected_port: int):
        self.BP = BP
        self.connected_port = connected_port
        BP.set_sensor_type(connected_port, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)

    def obstacle_detected(self) -> bool:
        try:
            distance = self.BP.get_sensor(self.connected_port)
            print('Sensor distance: {}'.format(distance))
            return distance < 12
        except brickpi3.SensorError:
            return False
