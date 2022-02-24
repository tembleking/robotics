#!/usr/bin/env python
import datetime
import math

from robotics.geometry import Location, Point


def print_location(location: Location):
    return (f'(x: {location.origin.x},'
            f' y: {location.origin.y},'
            f' deg: {location.angle_degrees()},'
            f' rad: {location.angle_radians()})')


print('##########################################')
print('Apartado 1')
door_in_world = Location.from_angle_degrees(Point(2.5, 10), 0)
door_seen_from_robot = Location.from_angle_degrees(Point(5.2, -3), -125)

robot_seen_from_door = door_seen_from_robot.inverse()
robot_in_world = robot_seen_from_door.seen_from_other_location(door_in_world)
print(f'robot_in_world: {print_location(robot_in_world)}')
print('##########################################')
print('')

print('##########################################')
print('Apartado 2')
painting_seen_from_robot = Location.from_angle_degrees(Point(4.17, 0.73), -35)
painting_in_world = painting_seen_from_robot.seen_from_other_location(robot_in_world)
print(f'painting_in_world: {print_location(painting_in_world)}')
print('##########################################')
print('')

print('##########################################')
print('Apartado 3')
world_seen_from_door = door_in_world.inverse()
painting_seen_from_door = painting_in_world.seen_from_other_location(world_seen_from_door)
print(f'painting_seen_from_door: {print_location(painting_seen_from_door)}')
print('##########################################')
print('')

print('##########################################')
print('Apartado 4')
print(f'radius of curvature: {door_seen_from_robot.radius_of_curvature()}')
print(f'angle of curvature: {math.degrees(door_seen_from_robot.angle_of_curvature())} degrees')
print(f'angular velocity: {door_seen_from_robot.angular_velocity_to_arrive_in(datetime.timedelta(seconds=8))}')
print(f'linear velocity: {door_seen_from_robot.linear_velocity_to_arrive_in(datetime.timedelta(seconds=8))}')

new_robot_seen_from_robot = door_seen_from_robot.new_location_after_rotating()
print(f'new_robot_seen_from_robot: {print_location(new_robot_seen_from_robot)}')

new_robot_in_world = new_robot_seen_from_robot.seen_from_other_location(robot_in_world)
print(f'new_robot_in_world: {print_location(new_robot_in_world)}')
print('##########################################')
print('')

print('##########################################')
print('Apartado 5')
new_robot_seen_from_robot_with_half_rotation = door_seen_from_robot.new_location_after_rotating(movement_ratio=0.5)
print(f'new_robot_seen_from_robot_with_half_rotation: {print_location(new_robot_seen_from_robot_with_half_rotation)}')

new_robot_in_world_with_half_rotation = new_robot_seen_from_robot_with_half_rotation.seen_from_other_location(
    robot_in_world)
print(f'new_robot_in_world_with_half_rotation: {print_location(new_robot_in_world_with_half_rotation)}')
print('##########################################')
