import math
import time

from robotics.geometry import Direction, Location, Point
from robotics.robot.robot import Robot


class Controller:
    def __init__(self, robot: Robot, polling_period: float, trajectory_generator, ball_following_speed_generator=None,
                 camera=None, obstacle_detector=None):
        self.robot = robot
        self.polling_period = polling_period
        self.visited_points = []
        self.trajectory_generator = trajectory_generator
        self.ball_following_speed_generator = ball_following_speed_generator
        self.obstacle_detector = obstacle_detector
        self.camera = camera
        self.last_point_distance = math.inf
        self.last_point_angle = math.inf
        self._has_arrived_angle = False

    def start(self):
        self.robot.start_odometry()
        while True:
            start = time.time()

            next_relative_location = self.get_next_relative_location()
            if next_relative_location is None:
                self.stop()
                return

            if self.has_arrived(next_relative_location):
                self.mark_point_as_visited()
                continue

            if self.has_arrived_angle(next_relative_location):
                if self.obstacle_detector.obstacle_detected():
                    self.last_point_distance = math.inf
                    self.last_point_angle = math.inf
                    self._has_arrived_angle = False
                    print('obstacle detected')
                    self.trajectory_generator.mark_wall_ahead()
                    continue

            v, w = self.get_next_velocities(next_relative_location)
            self.robot.set_speed(v, w)
            end_time = time.time()
            sleep_time = self.polling_period - (end_time - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def has_arrived_angle(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        self._has_arrived_angle = angle_to_arrive > self.last_point_angle or self._has_arrived_angle or self.last_point_angle == 0
        self.last_point_angle = angle_to_arrive
        return self._has_arrived_angle

    def has_arrived_distance(self, next_relative_location: Location) -> bool:
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = distance_to_arrive > self.last_point_distance and distance_to_arrive < 0.2
        self.last_point_distance = distance_to_arrive
        return has_arrived

    def has_arrived(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = self.has_arrived_angle(next_relative_location) and self.has_arrived_distance(
            next_relative_location)
        print('distance_to_arrive: %s, last_point_distance: %s, angle_to_arrive=%s, has_arrived=%s' % (
            distance_to_arrive, self.last_point_distance, angle_to_arrive, has_arrived))
        return has_arrived

    def mark_point_as_visited(self):
        self.last_point_distance = math.inf
        self.last_point_angle = math.inf
        self._has_arrived_angle = False
        self.trajectory_generator.mark_point_as_visited()

    def _position_to_cell(self, point: Point):
        x_cell = int(point.x * 1000 / 400)
        y_cell = int(point.y * 1000 / 400)
        return (x_cell, y_cell)

    def get_next_relative_location(self):
        next_point = self.trajectory_generator.next_absolute_point_to_visit()
        if next_point is None:
            return None

        current_location_seen_from_world = self.robot.location()
        print('Current location seen from world: %s %s' % (
            self._position_to_cell(current_location_seen_from_world.origin), current_location_seen_from_world))
        if current_location_seen_from_world is None:
            return None

        self.visited_points.append(current_location_seen_from_world)
        world_seen_from_current_location = current_location_seen_from_world.inverse()
        next_location_from_current_location = next_point.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location

    def get_next_velocities(self, next_relative_location: Location):
        if next_relative_location.angle_radians() > 0.025:
            return 0, 0.25
        if next_relative_location.angle_radians() < -0.025:
            return 0, -0.25
        return 0.1, 0

    def is_ball_in_claws(self) -> bool:
        if self.camera is None:
            print('warning: tried to access the camera without an actual camera')
            return False
        return self.camera.is_ball_within_claws()

    def stop(self):
        self.robot.stop_odometry()
        self.robot.set_speed(0, 0)
