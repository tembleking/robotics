import math
import time

from robotics.geometry import Direction
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot

distance_threshold = 0.05
angle_threshold = 2


class Controller:
    def __init__(self, odometry: Odometry, robot: Robot, polling_period: float, trajectory_generator):
        self.odometry = odometry
        self.robot = robot
        self.polling_period = polling_period
        self.visited_points = []
        self.trajectory_generator = trajectory_generator

    def start(self):
        self.odometry.start()
        while True:
            start = time.time()
            next_relative_location = self.get_next_relative_location()
            if next_relative_location is None:
                self.odometry.stop()
                return

            distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
            angle_to_arrive = next_relative_location.angle_degrees()
            has_arrived = distance_to_arrive <= distance_threshold and angle_to_arrive <= angle_threshold
            print('distance_to_arrive: %s, angle_to_arrive=%s, has_arrived=%s' % (
                distance_to_arrive, angle_to_arrive, has_arrived))
            if has_arrived:
                self.trajectory_generator.mark_point_as_visited()
                continue

            if angle_to_arrive <= angle_threshold and distance_to_arrive > distance_threshold:
                print('going straight (v: 1, w: 0)')
                self.robot.set_speed(1, 0)

            if distance_to_arrive <= distance_threshold and angle_to_arrive > angle_threshold:
                w = float('%.3f' % (math.pi / 2))
                self.robot.set_speed(0, w)
                print('turning (v: 0, w: %s)' % w)

            if distance_to_arrive > distance_threshold and angle_to_arrive > angle_threshold:
                w = float('%.3f' % (1 / next_relative_location.radius_of_curvature()))
                self.robot.set_speed(1,
                                     w)
                print('doing an arc (v: 1, w: %s)' % w)

            end_time = time.time()
            sleep_time = self.polling_period - (end_time - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_next_relative_location(self):
        next_point = self.trajectory_generator.next_absolute_point_to_visit()
        if next_point is None:
            return None

        current_location_seen_from_world = self.odometry.location()
        self.visited_points.append(current_location_seen_from_world)
        world_seen_from_current_location = current_location_seen_from_world.inverse()
        next_location_from_current_location = next_point.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location
