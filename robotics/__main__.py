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


def left_wheel(BP: brickpi3.BrickPi3) -> Motor:
    return Motor(BP, left_wheel_port)


def right_wheel(BP: brickpi3.BrickPi3) -> Motor:
    return Motor(BP, right_wheel_port)


def controller(BP: brickpi3.BrickPi3, trajectory: list):
    return Controller(
        odometry=odometry(BP),
        robot=robot(BP),
        polling_period=0.2,
        trajectory_generator=trajectory_generator(trajectory),
    )


def odometry(BP: brickpi3.BrickPi3):
    return Odometry(
        left_motor=left_wheel(BP),
        right_motor=right_wheel(BP),
        polling_period=0.01,
        wheel_radius=wheel_radius,
        axis_length=axis_length,
    )


def robot(BP: brickpi3.BrickPi3):
    return Robot(
        left_motor=left_wheel(BP),
        right_motor=right_wheel(BP),
        claw_motor=None,
        wheel_radius=wheel_radius,
        axis_length=axis_length,
    )


def trajectory_generator(trajectory: list):
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


def run():
    BP = brickpi3.BrickPi3()

    ctrl = controller(BP, trajectory=square_trajectory())
    ctrl.start()

    dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_square.csv')
    #display_visited_points_in_graph(ctrl.visited_points)

    time.sleep(15)
    ctrl = controller(BP, trajectory=eight_trajectory())
    ctrl.start()

    dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_eight.csv')
    #display_visited_points_in_graph(ctrl.visited_points)

    time.sleep(15)
    ctrl = controller(BP, trajectory=wheels_trajectory())
    ctrl.start()

    dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_wheel.csv')
    # display_visited_points_in_graph(ctrl.visited_points)