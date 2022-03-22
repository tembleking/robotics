import math

from robotics.geometry import Location, Direction
from robotics.robot.odometry import Odometry
from robotics.robot.robot import Robot


class Controller:
    def __init__(self, odometry: Odometry, robot: Robot):
        self.odometry = odometry
        self.robot = robot
        self.visited_points = []

    def set_next_relative_point_to_visit(self, next_location: Location):
        self.next_location_to_visit_seen_from_world = next_location

    def start(self):
        next_location_from_current_location = self.get_next_location_from_current_location()

        while True:
            distance_to_arrive = Direction(next_location_from_current_location.origin.x,
                                           next_location_from_current_location.origin.y).modulus()
            angle_to_arrive = next_location_from_current_location.angle_degrees()
            has_arrived = distance_to_arrive <= 0.01 and angle_to_arrive <= 2
            if has_arrived:
                break

            if angle_to_arrive <= 2 and distance_to_arrive > 0.01:
                self.robot.set_speed(15, 0)

            if distance_to_arrive <= 0.01 and angle_to_arrive > 2:
                self.robot.set_speed(0, float('%.3f' % (math.pi / 2)))

            if distance_to_arrive > 0.01 and angle_to_arrive > 2:
                self.robot.set_speed(15,
                                     float('%.3f' % (15 / next_location_from_current_location.radius_of_curvature())))
            next_location_from_current_location = self.get_next_location_from_current_location()

    def get_next_location_from_current_location(self):
        current_location_seen_from_world = self.odometry.location()
        self.visited_points.append(current_location_seen_from_world)
        world_seen_from_current_location = current_location_seen_from_world.inverse()
        next_location_from_current_location = self.next_location_to_visit_seen_from_world.seen_from_other_location(
            world_seen_from_current_location)
        return next_location_from_current_location
