
from robotics.geometry import Direction, Location, Point
from robotics.sensors import camera

class FinalTrajectorySpeedGenerator:
    def __init__(self, isWhite: bool, camera: camera):
        self._center_white = [
            Location.from_angle_degrees(Point(2, 2), None),        # Angle unknown at the beginning
            Location.from_angle_degrees(Point(2, 2), 90)
        ]
        
        self._center_black = [
            Location.from_angle_degrees(Point(0.8, 2), None),      # Angle unknown at the beginning
            Location.from_angle_degrees(Point(0.8, 2), 90)
        ]
        
        self._speeds_center = [
            (0.1, 0),
            (0, 0.25),
        ]
        
        # Trajectories initiates on the left 
        self._trajectory_white = [
            Location.from_angle_degrees(Point(2, 2), 135),      # Turn Left to make a diagonal
            Location.from_angle_degrees(Point(1.3, 2.6), 135),
            Location.from_angle_degrees(Point(1.3, 2.6), 90),   # Turn Right to leave the circuit
            Location.from_angle_degrees(Point(1.3, 3.2), 90),   # Leave
        ]
        
        self._trajectory_black = [
            Location.from_angle_degrees(Point(0.8, 2), 135),    # Turn Left to make a diagonal
            Location.from_angle_degrees(Point(0.2, 2.6), 135),
            Location.from_angle_degrees(Point(0.2, 2.6), 90),   # Turn Right to leave the circuit
            Location.from_angle_degrees(Point(0.2, 3.2), 90),   # Leave
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
        
        self._camera = camera
        self._door = None # None: Unkown, Other: ("left" | "right")
        self._isWhite = isWhite
        
    def getSpeed(self, current_location: Location):
        # Refactor: Make it multiprocess and instead of doing a state
        #           machine, do 2 points (go to center and look up)
        #           and return (0, 0) while the thread is executing.
        #           After it finishes, continue poping locations
        
        # Going to the center
        if not self._has_reached_center(): 
            location = self._get_next_center_location()
            
            if self._has_arrived(location):
                self._pop_next_center_location()
           
            # We still don't know the angle to reach
            if self.location[2] == None:
                self.location[2] = current_location[2]
                
            return self._current_speed()
        
        # Looking for the robots
        elif self._door == None:
            self._door = camera.get_homography_robot_position()
            if self._door == "right":
                for i in range(len(self._trajectory_white)):
                    self._trajectory_white[i] = Location(
                        self._trajectory_white[i][0] + 1.2, self._trajectory_white[i][1])
                    
                    self._trajectory_black[i] = Location(
                        self._trajectory_black[i][0] + 1.2, self._trajectory_black[i][1])
            return 0, 0
        
        # Leaving the circuit
        elif not self._has_left():
            location = self._get_next_circuit_location()
            
            if self._has_arrived(location):
                self._pop_next_circuit_location()
                self._pop_next_location_speed()
            
            return self._current_trajectory_speed()

        return None
    
    # Going to the Center
    def _get_next_center_location(self):
        if self._isWhite:
            return self._center_white[0]
        
        return self._center_black[0]
    
    def _pop_next_center_location(self):
        if self._isWhite:
            self._center_white.pop()
            return
        
        self._center_black.pop()
        
    def _pop_next_center_speed(self):
        self._speeds_center.pop()
            
    def _has_reached_center(self):
        if self._isWhite:
            return len(self._center_white) == 0
        
        return len(self._center_black) == 0
    
    # Leaving the Circuit
    def _get_next_circuit_location(self):
        if self._isWhite:
            return self._trajectory_white[0]
        
        return self._trajectory_black[0]
    
    def _pop_next_circuit_location(self):
        if self._isWhite:
            self._trajectory_white.pop()
            return
        
        self._trajectory_black.pop()
        
    def _has_left(self):
        if self._isWhite:
            return len(self._trajectory_white) == 0
        
        return len(self._center_black) == 0
    
    def _pop_next_location_speed(self):
        if self._door == "left":
            self._speeds_left.pop()
            return
        
        self._speeds_right.pop()
    
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
        print(
            '[HardcodedSpeedGenerator]: distance_to_arrive: %s, last_point_distance: %s, angle_to_arrive=%s, has_arrived=%s' % (
                distance_to_arrive, self.last_point_distance, angle_to_arrive, has_arrived))
        return has_arrived
    
    def _current_center_speed(self, current_location: Location) -> (float, float):
        return self._speeds_center[0]
    
    def _current_trajectory_speed(self) -> (float, float):
        if self._door == "left":
            return self._speeds_trajectory_left[0]
        
        return self._speeds_trajectory_right[0]
        