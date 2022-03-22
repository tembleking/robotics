from robotics.actuators import brickpi3
from robotics.actuators.motor import Motor
from robotics.geometry import Location, Point
from robotics.robot.controller import Controller
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot
from robotics.robot.trajectory_generator import TrajectoryGenerator

wheel_radius = 0.05  # FIXME: Use valid values
axis_length = 0.17  # FIXME: Use valid values
left_wheel_port = brickpi3.BrickPi3.PORT_1  # FIXME: Use the valid port
right_wheel_port = brickpi3.BrickPi3.PORT_2  # FIXME: Use the valid port


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
    return []


def dump_visited_points_to_csv_file(visited_points: [Location], file_name):
    with open(file=file_name, mode='w') as file:
        file.write('x,y,th\n')
        for point in visited_points:
            file.write(f'{point.origin.x},{point.origin.y},{point.angle_radians()}\n')


if __name__ == '__main__':
    BP = brickpi3.BrickPi3()

    ctrl = controller(BP, trajectory=square_trajectory())
    ctrl.start()

    dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points.csv')
