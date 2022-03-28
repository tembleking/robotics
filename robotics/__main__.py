import matplotlib.pyplot
import time

from robotics.actuators import brickpi3
from robotics.actuators.motor import Motor
from robotics.geometry import Location, Point
from robotics.paint import MatPlotLibPrinter, RobotPainter
from robotics.robot.controller import Controller
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot
from robotics.robot.trajectory_generator import TrajectoryGenerator

wheel_radius = 0.025
axis_length = 0.119
left_wheel_port = brickpi3.BrickPi3.PORT_B
right_wheel_port = brickpi3.BrickPi3.PORT_A


class Factory:
    def __init__(self, BP: brickpi3.BrickPi3):
        self.bp = BP
        self._odometry = None

    def left_wheel(self) -> Motor:
        return Motor(self.bp, left_wheel_port, motor_name='left_wheel')

    def right_wheel(self) -> Motor:
        return Motor(self.bp, right_wheel_port, motor_name='right_wheel')

    def controller(self, trajectory: list):
        return Controller(
            odometry=self.odometry(),
            robot=self.robot(),
            polling_period=0.2,
            trajectory_generator=self.trajectory_generator(trajectory),
            k_rho=0.1,
            k_alpha=0.2,
            k_beta=0.1,
        )

    def odometry(self):
        if self._odometry is None:
            self._odometry = Odometry(
                left_motor=self.left_wheel(),
                right_motor=self.right_wheel(),
                polling_period=0.01,
                wheel_radius=wheel_radius,
                axis_length=axis_length,
            )
        return self._odometry

    def robot(self):
        return Robot(
            left_motor=self.left_wheel(),
            right_motor=self.right_wheel(),
            claw_motor=None,
            wheel_radius=wheel_radius,
            axis_length=axis_length,
        )

    def trajectory_generator(self, trajectory: list):
        return TrajectoryGenerator(trajectory)


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


def eight_trajectory():
    return [
        Location.from_angle_degrees(Point(0, 0), -90),
        Location.from_angle_degrees(Point(0, 0.8), 90),
        Location.from_angle_degrees(Point(0, 1.6), -90),
        Location.from_angle_degrees(Point(0, 0.8), 90),
        Location.from_angle_degrees(Point(0, 0), -90),
    ]


def wheels_trajectory():
    return [
        Location.from_angle_degrees(Point(0, 0), 90),
        Location.from_angle_degrees(Point(0.17, 0.2), 9.6),
        Location.from_angle_degrees(Point(1.33, 0.39), 9.6),
        Location.from_angle_degrees(Point(1.33, -0.39), 90 - 9.6),
        Location.from_angle_degrees(Point(0.17, -0.2), 90 - 9.6),
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


def stop_robot(factory):
    factory.left_wheel().set_speed(0)
    factory.right_wheel().set_speed(0)
    try:
        factory.odometry().stop()
    except Exception as ex:
        print('exception captured while stopping odometry: %s' % ex)


def run():
    BP = brickpi3.BrickPi3()
    factory = Factory(BP)

    print('Starting robot')
    try:
        print('Starting square trajectory')
        ctrl = factory.controller(trajectory=square_trajectory())
        ctrl.start()

        dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_square.csv')
        # display_visited_points_in_graph(ctrl.visited_points)

        print('Waiting 15 seconds until next trajectory')
        time.sleep(15)

        print('Starting eight trajectory')
        ctrl = factory.controller(trajectory=eight_trajectory())
        ctrl.start()

        dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_eight.csv')
        # display_visited_points_in_graph(ctrl.visited_points)

        print('Waiting 15 seconds until next trajectory')
        time.sleep(15)

        print('Starting wheels trajectory')
        ctrl = factory.controller(trajectory=wheels_trajectory())
        ctrl.start()

        dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_wheel.csv')
        # display_visited_points_in_graph(ctrl.visited_points)
    except BaseException as e:
        print('captured exception: %s' % e)
        stop_robot(factory)
