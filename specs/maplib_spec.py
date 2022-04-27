import numpy as np
from hamcrest import equal_to, assert_that
from mamba import context, description, it

from robotics.robot.map import Map


def mapa0():
    return b"""\
3 3 400
0 0 0 0 0 0 0
0 1 1 1 1 1 0
0 1 1 1 1 1 0
0 1 1 1 1 1 0
0 1 1 1 1 1 0
0 1 1 1 1 1 0
0 0 0 0 0 0 0
"""


def mapa1():
    return b"""\
3 3 400
0 0 0 0 0 0 0
0 1 1 1 1 1 0
0 1 0 1 0 1 0
0 1 0 1 0 1 0
0 1 0 1 0 0 0
0 1 0 1 1 1 0
0 0 0 0 0 0 0
"""


def mapa2():
    return b"""\
7 5 400
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 0 0 0 0 1 1 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 0 1 0 0 0 0 0
0 1 1 1 1 1 0 1 0 1 1 1 1 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0
0 1 0 1 1 1 1 1 0 1 1 1 1 1 0
0 1 0 0 0 0 0 1 0 1 1 1 1 1 0
0 1 1 1 1 1 0 1 0 1 1 1 1 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
"""


def mapa3():
    return b"""\
8 5 400
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 0 0 0 0 0 0 1 0 1 0
0 1 1 1 1 1 0 1 1 1 1 1 1 1 0 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0 1 0
0 1 0 1 1 1 0 1 0 1 1 1 1 1 0 1 0
0 1 0 1 1 1 1 1 0 1 1 1 1 1 1 1 0
0 1 0 0 0 0 0 1 0 1 1 1 1 1 1 1 0
0 1 1 1 1 1 0 1 0 1 1 1 1 1 1 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
"""


with description('maplib', 'unit') as self:
    with it('should load the map correctly from a byte array'):
        map = Map(mapa0())
        assert (map.connectionMatrix == np.matrix([[0., 0., 0., 0., 0., 0., 0.],
                                                   [0., 1., 1., 1., 1., 1., 0.],
                                                   [0., 1., 1., 1., 1., 1., 0.],
                                                   [0., 1., 1., 1., 1., 1., 0.],
                                                   [0., 1., 1., 1., 1., 1., 0.],
                                                   [0., 1., 1., 1., 1., 1., 0.],
                                                   [0., 0., 0., 0., 0., 0., 0.]])).all()

    with it('should fill the cost matrix correctly'):
        map = Map(mapa0())
        map.fillCostMatrix(2, 2)
        assert (map.costMatrix == np.matrix([
            [4., 3., 2.],
            [3., 2., 1.],
            [2., 1., 0.]
        ])).all()

    with it('should fill the cost matrix correctly when the map has walls'):
        map = Map(mapa1())
        map.fillCostMatrix(2, 2)
        assert (map.costMatrix == np.matrix([
            [4., 3., 2.],
            [3., 2., 1.],
            [4., 1., 0.]
        ])).all()

    with it('should fill the cost matrix correctly when the map has walls'):
        map = Map(mapa2())
        map.fillCostMatrix(6, 1)
        assert (map.costMatrix == np.matrix([
            [13., 12., 11., 12., 11.],
            [14., 9., 10., 11., 10.],
            [15., 8., 9., 10., 9.],
            [8., 7., 6., 5., 8.],
            [3., 2., 3., 4., 7.],
            [2., 1., 2., 5., 6.],
            [1., 0., 1., 6., 7.],
        ])).all()

    with context('when retrieving the path to reach the goal'):
        with it('should return the correct path'):
            map = Map(mapa2())
            map.fillCostMatrix(6, 1)
            path = map.findPath((0, 0), (6, 1))
            assert_that(path, equal_to(
                [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1), (3, 1), (3, 2), (3, 3), (4, 3), (4, 2), (5, 2), (6, 2),
                 (6, 1)]
            ))

    with context("when there's no path to the goal"):
        with it('should return an empty path'):
            map = Map(mapa0())
            map.deleteConnection(0, 0, 0)
            map.deleteConnection(0, 0, 2)
            map.fillCostMatrix(2, 2)

            path = map.findPath((0, 0), (2, 2))
            assert_that(path, equal_to([]))

    with context('when there are modifications to the original map'):
        with it('generates a valid path'):
            map = Map(mapa3())
            map.deleteConnection(2, 2, 4)
            map.deleteConnection(2, 4, 2)

            path = map.findPath((2, 2), (7, 0))
            map.verbose = True
            map.drawMap()

            assert_that(path, equal_to(
                [(2, 2), (1, 2), (1, 1), (2, 1), (3, 1), (3, 2), (3, 3), (4, 3), (5, 3), (6, 3), (6, 2), (6, 1), (7, 1),
                 (7, 0)]))
