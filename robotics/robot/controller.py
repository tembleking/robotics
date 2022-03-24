import math
import time

from robotics.geometry import Direction
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot


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
            has_arrived = distance_to_arrive <= 0.01 and angle_to_arrive <= 2
            if has_arrived:
                self.trajectory_generator.mark_point_as_visited()
                continue

            if angle_to_arrive <= 2 and distance_to_arrive > 0.01:
                self.robot.set_speed(0.15, 0)

            if distance_to_arrive <= 0.01 and angle_to_arrive > 2:
                self.robot.set_speed(0, float('%.3f' % (math.pi / 2)))

            if distance_to_arrive > 0.01 and angle_to_arrive > 2:
                self.robot.set_speed(0.15,
                                     float('%.3f' % (0.15 / next_relative_location.radius_of_curvature())))

            time.sleep(self.polling_period - (time.time() - start))

    def get_next_relative_location(self):
        next_point = self.trajectory_generator.next_absolute_point_to_visit()
        if next_point is None:
            return None

        current_location_seen_from_world = self.odometry.location()
        self.visited_points.append(current_location_seen_from_world)
        world_seen_from_current_location = current_location_seen_from_world.inverse()
        next_location_from_current_location = next_point.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location
