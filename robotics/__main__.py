import matplotlib.pyplot
import time
import cv2

from robotics.actuators import brickpi3
from robotics.actuators.motor import Motor
from robotics.geometry import Location, Point
from robotics.paint import MatPlotLibPrinter, RobotPainter
from robotics.robot.controller import Controller
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot
from robotics.robot.trajectory_generator import TrajectoryGenerator
from robotics.robot.ball_following_speed_generator import BallFollowingSpeedGenerator
from robotics.sensors.camera import Camera

wheel_radius = 0.025
axis_length = 0.119
left_wheel_port = brickpi3.BrickPi3.PORT_B
right_wheel_port = brickpi3.BrickPi3.PORT_A
claw_port = brickpi3.BrickPi3.PORT_C


class Factory:
    def __init__(self, BP: brickpi3.BrickPi3):
        self.bp = BP
        self._odometry = None
        self._camera = None

    def left_wheel(self) -> Motor:
        return Motor(self.bp, left_wheel_port, motor_name='left_wheel')

    def right_wheel(self) -> Motor:
        return Motor(self.bp, right_wheel_port, motor_name='right_wheel')

    def claw(self) -> Motor:
        return Motor(self.bp, claw_port, motor_name='claw', motor_limit_dps=60)

    def controller(self, trajectory: list):
        return Controller(
            robot=self.robot(),
            polling_period=0.05,
            trajectory_generator=self.trajectory_generator(trajectory),
            ball_following_speed_generator=self.ball_following_speed_generator(),
            camera=self.camera(),
            k_rho=0.35,
            k_alpha=1,
            k_beta=0.5,
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
            odometry=self.odometry(),
            left_motor=self.left_wheel(),
            right_motor=self.right_wheel(),
            claw_motor=self.claw(),
            wheel_radius=wheel_radius,
            axis_length=axis_length,
        )

    def trajectory_generator(self, trajectory: list):
        return TrajectoryGenerator(trajectory)

    def camera(self):
        if self._camera is None:
            video_capture = cv2.VideoCapture(0)
            video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            self._camera = Camera(video_capture)
        return self._camera

    def ball_following_speed_generator(self):
        return BallFollowingSpeedGenerator(
            camera=self.camera(),
            area_goal=214,
            distance_goal=267,
            distance_damping=0.001,
            area_damping=0.001,
        )


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


def save_visited_points_in_graph(visited_points: list, trajectory, filename: str):
    painter = RobotPainter(MatPlotLibPrinter())
    for point in visited_points:
        painter.paint([point.origin.x, point.origin.y, point.angle_radians()])
    for point in trajectory:
        painter.paint([point.origin.x, point.origin.y, point.angle_radians()])
    matplotlib.pyplot.savefig(filename)


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
        print('Starting one-point trajectory')
        ctrl = factory.controller(trajectory=one_point())
        ctrl.start()

        dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_one_point.csv')
        # display_visited_points_in_graph(ctrl.visited_points)

        print('Waiting 15 seconds until next trajectory')
        time.sleep(15)

        print('Starting square trajectory')
        ctrl = factory.controller(trajectory=square_trajectory())
        ctrl.start()

        dump_visited_points_to_csv_file(ctrl.visited_points, 'visited_points_square.csv')
        # display_visited_points_in_graph(ctrl.visited_points)

        print('Waiting 15 seconds until next trajectory')
        time.sleep(15)

        print('Starting square trajectory without turns')
        ctrl = factory.controller(trajectory=square_trajectory_without_turns())
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
    finally:
        stop_robot(factory)
        dump_visited_points_to_csv_file(ctrl.visited_points, 'latest_run.csv')
        save_visited_points_in_graph(ctrl.visited_points, trajectory=Location.from_angle_degrees(Point(0, 0), 0),
                                     filename='latest_odometry.png')
