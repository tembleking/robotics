from robotics.actuators import brickpi3


class Sonar:
    def __init__(self, BP: brickpi3.BrickPi3, connected_port: int):
        self.BP = BP
        self.connected_port = connected_port
        BP.set_sensor_type(connected_port, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)

    def obstacle_detected(self) -> bool:
        try:
            distance = self.BP.get_sensor(self.connected_port)
            return 0 < distance < 12
        except brickpi3.SensorError:
            return False

    def distance(self) -> float:
        try:
            distance = 0
            for i in range(1000):
                distance += self.BP.get_sensor(self.connected_port)
            distance /= 1000
            print('Sonar distance: {}'.format(distance))
            return distance / 100
        except brickpi3.SensorError:
            return 0.
