import datetime
import math

from hamcrest import assert_that, is_, close_to
from mamba import description, it

from robotics.geometry import Point, Direction, Location, PolarCoordinates

with description('Geometry', 'unit') as self:
    with it('moves the point in the space'):
        p_in_a = Point(2, 3)

        displaced = p_in_a.displace(1, 1)

        assert_that(displaced, is_(Point(3, 4)))

    with it('rotates a point 90 degrees'):
        p_in_a = Point(1, 0)

        rotated = p_in_a.rotate_degrees(90)

        assert_that(rotated, is_(Point(0, 1)))

    with it('rotates a direction 90 degrees'):
        d = Direction(1, 1)

        rotated = d.rotate_degrees(90)

        assert_that(rotated, is_(Direction(-1, 1)))

    with it('rotates a point -90 degrees'):
        p_in_a = Point(1, 0)

        rotated = p_in_a.rotate_degrees(-90)

        assert_that(rotated, is_(Point(0, -1)))

    with it('rotates a direction -90 degrees'):
        d = Direction(1, 0)

        rotated = d.rotate_degrees(-90)

        assert_that(rotated, is_(Direction(0, -1)))

    with it('rotates a point pi radians'):
        p_in_a = Point(1, 0)

        rotated = p_in_a.rotate_radians(3.14159265)  # 180 degrees

        assert_that(rotated, is_(Point(-1, 0)))

    with it('rotates a direction pi radians'):
        d = Direction(1, 0)

        rotated = d.rotate_radians(3.14159265)  # 180 degrees

        assert_that(rotated, is_(Direction(-1, 0)))

    with it('normalizes the direction'):
        d = Direction(1, 1)

        normalized = d.normalize()

        assert_that(normalized, is_(Direction(math.cos(math.pi / 4), math.cos(math.pi / 4))))

    with it('creates the location correctly'):
        b = Location(origin=Point(5, 6), x_axis=Direction(1, 2), y_axis=Direction(3, 4))

        assert_that(b.x_axis, is_(Direction(1, 2).normalize()))
        assert_that(b.y_axis, is_(Direction(3, 4).normalize()))
        assert_that(b.origin, is_(Point(5, 6)))

    with it('calculates the reverse location correctly'):
        b = Location.from_angle_degrees(Point(1, 1), 90)

        assert_that(b.inverse(), is_(Location.from_angle_degrees(Point(-1, 1), -90)))

    with it('calculates the location as seen from other location'):
        a = Location.from_angle_degrees(Point(0, 0), 5)
        b = Location.from_angle_degrees(Point(0, 0), 30)

        seen = b.seen_from_other_location(a)
        assert_that(seen, is_(Location.from_angle_degrees(Point(0, 0), 35)))

    with it('retrieves the angle in radians of a location'):
        location = Location.from_angle_radians(Point(0, 0), math.pi / 4)

        assert_that(location.angle_radians(), is_(math.pi / 4))

    with it('retrieves the angle in degrees of a location'):
        location = Location.from_angle_radians(Point(0, 0), math.pi / 4)

        assert_that(location.angle_degrees(), is_(45))

    with it('sees a point from another location defined with axis'):
        # Graphical example: https://imgur.com/a/yKEgRI5
        p_in_a = Point(math.cos(math.pi / 4), math.cos(math.pi / 4))

        a_in_w = Location(origin=Point(3, 2), x_axis=Direction(1, 1), y_axis=Direction(-1, 1))
        p_in_w = p_in_a.seen_from_other_location(other=a_in_w)

        assert_that(p_in_w, is_(Point(3, 3)))

    with it('sees a point from another location defined with angle'):
        # Graphical example: https://imgur.com/a/yKEgRI5
        p_in_a = Point(math.cos(math.pi / 4), math.cos(math.pi / 4))

        a_in_w = Location.from_angle_degrees(origin=Point(3, 2), angle=45)
        p_in_w = p_in_a.seen_from_other_location(other=a_in_w)

        assert_that(p_in_w, is_(Point(3, 3)))

    with it('is able to calculate the Radius of curvature'):
        y_in_x = Location.from_angle_degrees(Point(5.2, -3), -125)

        radius = y_in_x.radius_of_curvature()

        assert_that(radius, is_(close_to(-6.0066666, 0.0000001)))

    with it('is able to calculate the Angle of curvature'):
        y_in_x = Location.from_angle_degrees(Point(5.2, -3), -125)

        angle = y_in_x.angle_of_curvature()

        assert_that(angle, is_(close_to(-1.046556644263951, 0.00000000001)))

    with it('calculates the angular velocity when moving to the location'):
        location = Location.from_angle_degrees(Point(5.2, -3), -125)

        angular_speed = location.angular_velocity_to_arrive_in(datetime.timedelta(seconds=8))

        assert_that(angular_speed, is_(close_to(-0.13081958053299386, 0.00000000001)))

    with it('calculates the linear velocity when moving to the location'):
        location = Location.from_angle_degrees(Point(5.2, -3), -125)

        angular_speed = location.linear_velocity_to_arrive_in(datetime.timedelta(seconds=8))

        assert_that(angular_speed, is_(close_to(0.7857896137348499, 0.00000000001)))

    with it('calculates the new location after rotating to it'):
        location = Location.from_angle_degrees(Point(5.2, -3), -125)

        new_location = location.new_location_after_rotating()

        assert_that(new_location, is_(Location.from_angle_degrees(Point(5.2, -3), -59.963278737698666)))

    with it('calculates the new location after partially rotating to it'):
        location = Location.from_angle_degrees(Point(5.2, -3), -125)

        new_location = location.new_location_after_rotating(movement_ratio=0.5)

        assert_that(new_location, is_(Location.from_angle_degrees(Point(x=3.001666203960727, y=-0.803778579801407),
                                                                  -29.981639368849333)))

    with it('finds the distance between two points'):
        a = Point(1, 1)
        b = Point(4, 5)

        assert_that(a.distance_to(b), is_(5))

    with it ('returns the correct polar coodinates'):
        location = Location.from_angle_degrees(Point(-1, -1), 30)
        polar_location = PolarCoordinates(location)

        assert_that(polar_location.rho, is_(close_to(math.sqrt(2), 0.0000001)))
        assert_that(polar_location.beta, is_(close_to(math.radians(45) , 0.0000001)))
        assert_that(polar_location.alpha, is_(close_to(math.radians(15), 0.0000001)))

    with it('returns the correct polar coodinates'):
        location = Location.from_angle_degrees(Point(1, 1), -150)
        polar_location = PolarCoordinates(location)

        assert_that(polar_location.rho, is_(close_to(math.sqrt(2), 0.0000001)))
        assert_that(polar_location.beta, is_(close_to(math.radians(-135) , 0.0000001)))
        assert_that(polar_location.alpha, is_(close_to(math.radians(15), 0.0000001)))
