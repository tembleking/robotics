import math

from expects import equal, expect
from mamba import description, it

from geometry import Point, Direction, Basis

with description('coordinates') as self:
    with it('moves the point in the space'):
        p_in_a = Point(2, 3)

        displaced = p_in_a.displace(1, 1)

        expect(displaced).to(equal(Point(3, 4)))

    with it('rotates a point 90 degrees'):
        p_in_a = Point(1, 0)

        rotated = p_in_a.rotate_degrees(90)

        expect(rotated).to(equal(Point(0, 1)))

    with it('rotates a direction 90 degrees'):
        d = Direction(1, 1)

        rotated = d.rotate_degrees(90)

        expect(rotated).to(equal(Direction(-1, 1)))

    with it('rotates a point -90 degrees'):
        p_in_a = Point(1, 0)

        rotated = p_in_a.rotate_degrees(-90)

        expect(rotated).to(equal(Point(0, -1)))

    with it('rotates a direction -90 degrees'):
        d = Direction(1, 0)

        rotated = d.rotate_degrees(-90)

        expect(rotated).to(equal(Direction(0, -1)))

    with it('rotates a point pi radians'):
        p_in_a = Point(1, 0)

        rotated = p_in_a.rotate_radians(3.14159265)  # 180 degrees

        expect(rotated).to(equal(Point(-1, 0)))

    with it('rotates a direction pi radians'):
        d = Direction(1, 0)

        rotated = d.rotate_radians(3.14159265)  # 180 degrees

        expect(rotated).to(equal(Direction(-1, 0)))

    with it('normalizes the direction'):
        d = Direction(1, 1)

        normalized = d.normalize()

        expect(normalized).to(equal(Direction(math.cos(math.pi / 4), math.cos(math.pi / 4))))

    with it('creates the basis correctly'):
        b = Basis(origin=Point(5, 6), x_axis=Direction(1, 2), y_axis=Direction(3, 4))

        expect(b.x_axis).to(equal(Direction(1, 2).normalize()))
        expect(b.y_axis).to(equal(Direction(3, 4).normalize()))
        expect(b.origin).to(equal(Point(5, 6)))

    with it('calculates the reverse basis correctly'):
        b = Basis.from_angle_degrees(Point(1, 1), 90)

        expect(b.inverse()).to(equal(Basis.from_angle_degrees(Point(-1, 1), -90)))

    with it('calculates the basis as seen from other basis'):
        a = Basis.from_angle_degrees(Point(0, 0), 5)
        b = Basis.from_angle_degrees(Point(0, 0), 30)

        seen = b.seen_from_other_basis(a)
        expect(seen).to(equal(Basis.from_angle_degrees(Point(0, 0), 35)))

    with it('retrieves the angle in radians of a basis'):
        basis = Basis.from_angle_radians(Point(0, 0), math.pi / 4)

        expect(basis.angle_radians()).to(equal(math.pi / 4))

    with it('retrieves the angle in degrees of a basis'):
        basis = Basis.from_angle_radians(Point(0, 0), math.pi / 4)

        expect(basis.angle_degrees()).to(equal(45))

    with it('sees a point from another basis defined with axis'):
        # Graphical example: https://imgur.com/a/yKEgRI5
        p_in_a = Point(math.cos(math.pi / 4), math.cos(math.pi / 4))

        a_basis = Basis(origin=Point(3, 2), x_axis=Direction(1, 1), y_axis=Direction(-1, 1))
        p_in_w = p_in_a.seen_from_other_basis(other=a_basis)

        expect(p_in_w).to(equal(Point(3, 3)))

    with it('sees a point from another basis defined with angle'):
        # Graphical example: https://imgur.com/a/yKEgRI5
        p_in_a = Point(math.cos(math.pi / 4), math.cos(math.pi / 4))

        a_basis = Basis.from_angle_degrees(origin=Point(3, 2), angle=45)
        p_in_w = p_in_a.seen_from_other_basis(other=a_basis)

        expect(p_in_w).to(equal(Point(3, 3)))
