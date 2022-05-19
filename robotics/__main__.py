import math
import time

import cv2
import matplotlib.pyplot

from robotics.actuators import brickpi3
from robotics.actuators.motor import Motor
from robotics.geometry import Location, Point
from robotics.paint import MatPlotLibPrinter, RobotPainter
from robotics.robot.ball_following_speed_generator import BallFollowingSpeedGenerator
from robotics.robot.controller import Controller
from robotics.robot.hardcoded_speed_generator import HardcodedSpeedGenerator
from robotics.robot.map import Map
from robotics.robot.obstacle_trajectory_speed_generator import ObstacleTrajectorySpeedGenerator
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot
from robotics.robot.trajectory_generator import TrajectoryGenerator
from robotics.sensors.camera import Camera
from robotics.sensors.gyro import Gyro
from robotics.sensors.light import Light
from robotics.sensors.sonar import Sonar

wheel_radius = 0.0275
axis_length = 0.115
left_wheel_port = brickpi3.BrickPi3.PORT_B
right_wheel_port = brickpi3.BrickPi3.PORT_A
claw_port = brickpi3.BrickPi3.PORT_C
sonar_port = brickpi3.BrickPi3.PORT_3
light_sensor_port = brickpi3.BrickPi3.PORT_4
compass_port = brickpi3.BrickPi3.PORT_1
gyro_port = brickpi3.BrickPi3.PORT_1
white_initial_odometry = [0.6, 2.8, -math.pi / 2]
black_initial_odometry = [2.2, 2.8, -math.pi / 2]
white_destination_cell = [3, 3]
black_destination_cell = [3, 3]


class Factory:
    def __init__(self, BP: brickpi3.BrickPi3, white_map: bytes, black_map: bytes):
        self._robot = None
        self.bp = BP
        self._odometry = None
        self._camera = None
        self._map = None
        self._destination = None
        self._light_sensor_is_white = None

    def light_sensor_is_white(self):
        if self._light_sensor_is_white is None:
            light_sensor = Light(self.bp, light_sensor_port)
            time.sleep(1)
            self._light_sensor_is_white = light_sensor.is_white()
        print('Is light sensor white: %s' % self._light_sensor_is_white)
        return self._light_sensor_is_white

    def sonar(self):
        return Sonar(self.bp, sonar_port)

    def left_wheel(self) -> Motor:
        return Motor(self.bp, left_wheel_port, motor_name='left_wheel')

    def right_wheel(self) -> Motor:
        return Motor(self.bp, right_wheel_port, motor_name='right_wheel')

    def claw(self) -> Motor:
        return Motor(self.bp, claw_port, motor_name='claw', motor_limit_dps=60)

    def speed_generators(self) -> list:
        return [
            HardcodedSpeedGenerator(),
            self.obstacle_trajectory_generator(),
            self.ball_following_speed_generator(),
        ]

    def obstacle_trajectory_generator(self):
        return ObstacleTrajectorySpeedGenerator(self.trajectory_generator(), obstacle_detector=self.sonar())

    def controller(self):

        return Controller(
            robot=self.robot(),
            polling_period=0.01,
            speed_generators=self.speed_generators(),
        )

    def odometry(self):
        if self._odometry is None:
            self._odometry = Odometry(
                left_motor=self.left_wheel(),
                right_motor=self.right_wheel(),
                polling_period=0.01,
                wheel_radius=wheel_radius,
                axis_length=axis_length,
                initial_location=self.initial_location(),
                gyro=self.gyro()
            )
        return self._odometry

    def gyro(self):
        initial_angle = self.initial_location()[2]
        return Gyro(BP=self.bp, port=gyro_port, offset=initial_angle)

    def robot(self):
        if not self._robot:
            self._robot = Robot(
                odometry=self.odometry(),
                left_motor=self.left_wheel(),
                right_motor=self.right_wheel(),
                claw_motor=self.claw(),
                wheel_radius=wheel_radius,
                axis_length=axis_length,
            )
        return self._robot

    def map(self):
        if not self._map:
            self._map = self.load_white_map() if self.light_sensor_is_white() else self.load_black_map()
        return self._map

    def trajectory_generator(self):
        return TrajectoryGenerator(self.robot(), self.map(),
                                   destination=self.destination())

    def destination(self):
        if not self._destination:
            self._destination = Point(white_destination_cell[0],
                                      white_destination_cell[1]) if self.light_sensor_is_white() else Point(
                black_destination_cell[0], black_destination_cell[1])
        return self._destination

    def camera(self):
        if self._camera is None:
            video_capture = cv2.VideoCapture(0)
            video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            self._camera = Camera(video_capture)
        return self._camera

    def ball_following_speed_generator(self):
        return BallFollowingSpeedGenerator(
            camera=self.camera(),
            robot=self.robot(),
            area_goal=214,
            distance_goal=267,
            distance_damping=0.001,
            area_damping=0.001,
        )

    def load_white_map(self) -> Map:
        return Map(white_map_contents())

    def load_black_map(self) -> Map:
        return Map(black_map_contents())

    def initial_location(self) -> list:
        return white_initial_odometry if self.light_sensor_is_white() else black_initial_odometry


def one_point():
    return [
        Location.from_angle_degrees(Point(5, 0), 0),
    ]


def square_trajectory():
    return [
        Location.from_angle_degrees(Point(0.8, 0), 0),
        Location.from_angle_degrees(Point(0.8, 0), 90),
        Location.from_angle_degrees(Point(0.8, 1.2), 90),
        Location.from_angle_degrees(Point(0.8, 1.2), 180),
        Location.from_angle_degrees(Point(0, 1.2), 180),
        Location.from_angle_degrees(Point(0, 1.2), -90),
        Location.from_angle_degrees(Point(0, 0), -90),
        Location.from_angle_degrees(Point(0, 0), 0),
    ]


def square_trajectory_without_turns():
    return [
        # Location.from_angle_degrees(Point(0.8, 0), 0),
        Location.from_angle_degrees(Point(0.8, 0), 90),
        # Location.from_angle_degrees(Point(0.8, 1.2), 90),
        Location.from_angle_degrees(Point(0.8, 1.2), 180),
        # Location.from_angle_degrees(Point(0, 1.2), 180),
        Location.from_angle_degrees(Point(0, 1.2), -90),
        # Location.from_angle_degrees(Point(0, 0), -90),
        Location.from_angle_degrees(Point(0, 0), 0),
    ]


def eight_trajectory():
    return [
        Location.from_angle_degrees(Point(0, 0), -90),
        Location.from_angle_degrees(Point(0.4, -0.4), 0),
        Location.from_angle_degrees(Point(0.8, 0), 90),
        Location.from_angle_degrees(Point(1.2, 0.4), 0),
        Location.from_angle_degrees(Point(1.6, 0), -90),
        Location.from_angle_degrees(Point(1.2, -0.4), -180),
        Location.from_angle_degrees(Point(0.8, 0), 90),
        Location.from_angle_degrees(Point(0.4, 0.4), -180),
        Location.from_angle_degrees(Point(0, 0), -90),
    ]


def wheels_trajectory():
    return [
        Location.from_angle_degrees(Point(0, 0), 90),
        Location.from_angle_degrees(Point(0.17, 0.2), 9.6),
        Location.from_angle_degrees(Point(1.33, 0.39), 9.6),
        Location.from_angle_degrees(Point(1.6, 0), -90),
        Location.from_angle_degrees(Point(1.33, -0.39), 9.6 - 180),
        Location.from_angle_degrees(Point(0.17, -0.2), 9.6 - 180),
        Location.from_angle_degrees(Point(0, 0), 90),
    ]


def dump_visited_points_to_csv_file(visited_points: [Location], file_name):
    with open(file=file_name, mode='w') as file:
        file.write('x,y,th\n')
        for point in visited_points:
            file.write('%s,%s,%s\n' % (point.origin.x, point.origin.y, point.angle_radians()))


def display_visited_points_in_graph(visited_points: list):
    painter = RobotPainter(MatPlotLibPrinter())
    for point in visited_points:
        painter.paint([point.origin.x, point.origin.y, point.angle_radians()])
    matplotlib.pyplot.show()


def save_visited_points_in_graph(visited_points: list, filename: str):
    painter = RobotPainter(MatPlotLibPrinter())
    for point in visited_points:
        painter.paint([point.origin.x, point.origin.y, point.angle_radians()])
    matplotlib.pyplot.savefig(filename)


def stop_robot(factory):
    factory.left_wheel().set_speed(0)
    factory.right_wheel().set_speed(0)
    try:
        factory.odometry().stop()
    except Exception as ex:
        print('exception captured while stopping odometry: %s' % ex)


def map_contents():
    return b"""\
8 5 400
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 0 0 0 0 0 0 1 0 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0 1 0
0 1 0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 0 0 0 0 0 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 0 1 1 1 1 1 1 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
"""


def white_map_contents():
    return b"""\
7 7 400
0 0 0 1 0 0 0 0 0 0 0 0 0 0 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 1 0 1 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 1 0 1 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 0 0 0 0 0 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
"""


def black_map_contents():
    return b"""\
7 7 400
0 0 0 0 0 0 0 0 0 0 0 1 0 0 0
0 1 1 1 1 1 1 1 0 1 1 1 1 1 0
0 1 1 1 1 1 1 1 0 1 1 1 1 1 0
0 1 1 1 1 1 1 1 0 1 1 0 1 1 0
0 1 1 1 1 1 1 1 0 1 1 1 1 1 0
0 1 1 1 1 1 1 1 0 1 1 1 1 1 0
0 1 1 1 1 1 1 1 0 1 1 1 1 1 0
0 1 1 1 1 1 1 1 0 1 1 0 1 1 0
0 1 0 0 0 0 0 1 0 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
"""


def run():
    BP = brickpi3.BrickPi3()
    factory = Factory(BP, white_map=white_map_contents(), black_map=black_map_contents())

    try:
        print('Starting robot')
        voltage_battery = BP.get_voltage_battery()
        print('Voltage: %s' % voltage_battery)
        if voltage_battery < 6:
            print('Battery voltage is low. Please change the batteries.')
            return
        time.sleep(1)
        print('Initial location: ', factory.odometry().location())
        ctrl = factory.controller()
        input('Press enter to start')
        ctrl.start()
    #     print('Starting one-point trajectory')
    #     ctrl = factory.controller()
    #     ctrl.start()
    #
    #     dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_one_point.csv')
    #     # display_visited_points_in_graph(ctrl.visited_points)
    #
    #     print('Waiting 15 seconds until next trajectory')
    #     time.sleep(15)
    #
    #     print('Starting square trajectory')
    #     ctrl = factory.controller()
    #     ctrl.start()
    #
    #     dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_square.csv')
    #     # display_visited_points_in_graph(ctrl.visited_points)
    #
    #     print('Waiting 15 seconds until next trajectory')
    #     time.sleep(15)
    #
    #     print('Starting square trajectory without turns')
    #     ctrl = factory.controller()
    #     ctrl.start()
    #
    #     dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_square.csv')
    #     # display_visited_points_in_graph(ctrl.visited_points)
    #
    #     print('Waiting 15 seconds until next trajectory')
    #     time.sleep(15)
    #
    #     print('Starting eight trajectory')
    #     ctrl = factory.controller()
    #     ctrl.start()
    #
    #     dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_eight.csv')
    #     # display_visited_points_in_graph(ctrl.visited_points)
    #
    #     print('Waiting 15 seconds until next trajectory')
    #     time.sleep(15)
    #
    #     print('Starting wheels trajectory')
    #     ctrl = factory.controller()
    #     ctrl.start()
    #
    #     dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_wheel.csv')
    #     # display_visited_points_in_graph(ctrl.visited_points)
    except BaseException as e:
        print('captured exception: %s' % e)
    finally:
        stop_robot(factory)
        BP.reset_all()
        print('Saving odometry...')
        # dump_visited_points_to_csv_file(ctrl.visited_points, 'latest_run.csv')
        save_visited_points_in_graph(ctrl.visited_points[::10], filename='latest_odometry.png')
        print('Saving map...')
        robot_points = [[point.origin.x * 1000, point.origin.y * 1000, point.angle_radians()] for point in
                        ctrl.visited_points[::10]]
        factory.map().drawMap(robotPosVectors=robot_points, saveSnapshot=True)
