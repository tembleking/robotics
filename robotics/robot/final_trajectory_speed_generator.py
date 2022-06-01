import math
import time
import traceback
from typing import Tuple
from robotics.geometry import Direction, Location, Point
from robotics.robot.robot import Robot
from robotics.sensors.camera import Camera
from robotics.sensors.sonar import Sonar

MAX_ANGULAR_SPEED = 0.25
MAX_LINEAL_SPEED = 0.1


class FinalTrajectorySpeedGenerator:
    def __init__(self, isWhite: bool, camera: Camera, sonar: Sonar, robot: Robot):
        self._sonar = sonar
        self._robot = robot
        self._center_white = [  # Angle unknown at the beginning
            Location.from_angle_degrees(Point(2, 2), 90)
        ]

        self._center_black = [  # Angle unknown at the beginning
            Location.from_angle_degrees(Point(0.8, 2), 90)
        ]

        self._speeds_center = [
            (0, MAX_ANGULAR_SPEED),
            (0.1, 0),
            (0, MAX_ANGULAR_SPEED)
        ]

        # Trajectories initiates on the left 
        self._trajectory_left_white = [
            Location.from_angle_degrees(Point(2, 2), 135),  # Turn Left to make a diagonal
            Location.from_angle_degrees(Point(1.5, 2.6), 135),
            Location.from_angle_degrees(Point(1.5, 2.6), 90),  # Turn Right to leave the circuit
            Location.from_angle_degrees(Point(1.5, 3.2), 90),  # Leave
        ]

        self._trajectory_right_white = [
            Location.from_angle_degrees(Point(2, 2), 45),  # Turn Left to make a diagonal
            Location.from_angle_degrees(Point(2.5, 2.6), 45),
            Location.from_angle_degrees(Point(2.5, 2.6), 90),  # Turn Right to leave the circuit
            Location.from_angle_degrees(Point(2.5, 3.2), 90),  # Leave
        ]

        self._speeds_trajectory_left = [
            (0, 0.25),
            (0.1, 0),
            (0.0, -0.25),
            (0.1, 0),
        ]

        self._speeds_trajectory_right = [
            (0, -0.25),
            (0.1, 0),
            (0.0, 0.25),
            (0.1, 0),
        ]
        self.last_point_distance = math.inf
        self.last_point_angle = math.inf
        self._has_arrived_angle_var = False

        self._camera = camera
        self._door = None  # None: Unkown, Other: ("left" | "right")
        self._isWhite = isWhite
        self.is_init = False
        self._homography_done = False

        if (not isWhite):
            for i in range(len(self._trajectory_left_white)):
                self._trajectory_left_white[i] = Location.from_angle_degrees(Point(
                    self._trajectory_left_white[i].origin.x - 1.2,
                    self._trajectory_left_white[i].origin.y
                ), self._trajectory_left_white[i].angle_degrees())

                self._trajectory_right_white[i] = Location.from_angle_degrees(Point(
                    self._trajectory_right_white[i].origin.x - 1.2,
                    self._trajectory_right_white[i].origin.y
                ), self._trajectory_right_white[i].angle_degrees())

    def get_speed(self, current_location: Location):
        # Refactor: Make it multiprocess and instead of doing a state
        #           machine, do 2 points (go to center and look up)
        #           and return (0, 0) while the thread is executing.
        #           After it finishes, continue poping locations

        # If first time, init path
        if not self.is_init:
            location = self._get_next_center_location()
            target_angle = math.atan2(location.origin.y - current_location.origin.y,
                                      location.origin.x - current_location.origin.x)
            angle = target_angle - current_location.angle_radians()
            if 0 <= angle <= math.pi or \
                    angle < -math.pi:
                self._speeds_center[2] = (self._speeds_center[2][0], -self._speeds_center[2][1])
            else:
                self._speeds_center[0] = (self._speeds_center[0][0], -self._speeds_center[0][1])
            if self._isWhite:
                self._center_white = [Location.from_angle_radians(current_location.origin, target_angle),
                                      Location.from_angle_radians(Point(2, 2), target_angle)] + \
                                     self._center_white
                print(self._center_white)
            else:
                self._center_black = [Location.from_angle_radians(current_location.origin, target_angle),
                                      Location.from_angle_radians(Point(0.8, 2), target_angle)] + \
                                     self._center_black
                print(self._center_black)
            self.is_init = True

        # Going to the center
        if not self._has_reached_center():
            location = self._get_next_center_location()
            relative_location = self._get_next_relative_location(current_location, location)

            if self._has_arrived(relative_location):
                print("[FinalGenerator]: (Center) has arrived")
                self._reset_last_positions()
                self._pop_next_center_location()
                return self._pop_next_center_speed()

            return self._current_center_speed(current_location)
        if not self._homography_done:
            self._homography_done = True
            return 0, 0
        # Looking for the robots
        elif self._door == None:
            time.sleep(5)
            try:
                self._recalculate_y_position_with_sonar()
                self._door = self._camera.get_homography_robot_position()
            except Exception:
                print(traceback.format_exc())
            if self._door is None:
                return 0, 0
            self._door = "right" if self._door is None else self._door
            return 0, 0

        # Leaving the circuit
        elif not self._has_left():
            location = self._get_next_circuit_location()
            relative_location = self._get_next_relative_location(current_location, location)

            if self._has_arrived(relative_location):
                print("[FinalGenerator]: (Leaving) has arrived")
                self._reset_last_positions()
                self._pop_next_circuit_location()
                return self._pop_next_location_speed()

            return self._current_trajectory_speed()
        print("Es este")
        return None

    def _reset_last_positions(self):
        self.last_point_distance = math.inf
        self.last_point_angle = math.inf
        self._has_arrived_angle_var = False

    # Going to the Center
    def _get_next_center_location(self):
        if self._isWhite:
            return self._center_white[0]

        return self._center_black[0]

    def _pop_next_center_location(self):
        if self._isWhite:
            self._center_white.pop(0)
            return

        self._center_black.pop(0)

    def _pop_next_center_speed(self) -> Tuple[float, float]:
        return self._speeds_center.pop(0)

    def _has_reached_center(self):
        if self._isWhite:
            return len(self._center_white) == 0

        return len(self._center_black) == 0

    # Leaving the Circuit
    def _get_next_circuit_location(self):
        if self._door == "left":
            return self._trajectory_left_white[0]

        return self._trajectory_right_white[0]

    def _pop_next_circuit_location(self):
        if self._door == "left":
            self._trajectory_left_white.pop(0)
            return

        self._trajectory_right_white.pop(0)

    def _has_left(self):
        if self._door == "left":
            return len(self._trajectory_left_white) == 0

        return len(self._trajectory_right_white) == 0

    def _pop_next_location_speed(self):
        if self._door == "left":
            return self._speeds_trajectory_left.pop(0)
        return self._speeds_trajectory_right.pop(0)

    def _current_center_speed(self, current_location: Location) -> Tuple[float, float]:
        return self._speeds_center[0]

    def _current_trajectory_speed(self) -> Tuple[float, float]:
        if self._door == "left":
            return self._speeds_trajectory_left[0]

        return self._speeds_trajectory_right[0]

    # Funtions from Harcoded Speed Generator
    def _has_arrived_angle(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        self._has_arrived_angle_var = (angle_to_arrive > self.last_point_angle and
                                       angle_to_arrive < 0.1) or self._has_arrived_angle_var or angle_to_arrive == 0 or self.last_point_angle == 0
        self.last_point_angle = angle_to_arrive
        return self._has_arrived_angle_var

    def _has_arrived_distance(self, next_relative_location: Location) -> bool:
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = distance_to_arrive > self.last_point_distance and distance_to_arrive < 0.1
        self.last_point_distance = distance_to_arrive
        return has_arrived

    def _has_arrived(self, next_relative_location: Location) -> bool:
        angle_to_arrive = abs(next_relative_location.angle_radians())
        distance_to_arrive = Direction(next_relative_location.origin.x, next_relative_location.origin.y).modulus()
        has_arrived = self._has_arrived_angle(next_relative_location) and self._has_arrived_distance(
            next_relative_location)
        if has_arrived:
            print(
                '[FinalGenerator]: distance_to_arrive: %s, last_point_distance: %s, angle_to_arrive=%s, has_arrived=%s' % (
                    distance_to_arrive, self.last_point_distance, angle_to_arrive, has_arrived))
        return has_arrived

    def _get_next_relative_location(self, current_location: Location, next_location: Location) -> Location:
        if next_location is None:
            return None

        world_seen_from_current_location = current_location.inverse()
        next_location_from_current_location = next_location.seen_from_other_location(world_seen_from_current_location)
        return next_location_from_current_location

    def _recalculate_y_position_with_sonar(self):
        robot_location = self._robot.location()
        self._robot.set_speed(0, 0)
        new_location = Location.from_angle_radians(Point(robot_location.origin.x, 2.8 - self._sonar.distance()),
                                                   robot_location.angle_radians())

        print("[FinalGenerator]: New location {}".format(new_location))
        self._robot.set_location(new_location)
