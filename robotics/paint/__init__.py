from enum import IntEnum

import matplotlib
import numpy


class Size(IntEnum):
    SMALL = 1
    BIG = 5


class Color(IntEnum):
    BLACK = 0
    OTHER = 1


class MatPlotLibPrinter:
    def __init__(self):
        pass

    def print(self, *args):
        matplotlib.pyplot.plot(*args)


class RobotPainter:
    def __init__(self, printer, size=Size.SMALL, color=Color.BLACK):
        self.printer = printer
        self.largo = 0.1 * size.value
        self.corto = 0.05 * size.value
        self.descentre = 0.01 * size.value
        self.color = color.value

    def paint(self, axis_location):
        right_back = numpy.array([-self.largo, -self.corto, 1])
        left_back = numpy.array([-self.largo, self.corto, 1])
        right_front = numpy.array([self.largo, -self.corto, 1])
        left_front = numpy.array([self.largo, self.corto, 1])

        frontal_of_robot = numpy.array([self.largo, 0, 1])
        tita = axis_location[2]
        Hwe = numpy.array([[numpy.cos(tita), -numpy.sin(tita), axis_location[0]],
                           [numpy.sin(tita), numpy.cos(tita), axis_location[1]],
                           [0, 0, 1]])
        Hec = numpy.array([[1, 0, self.descentre],
                           [0, 1, 0],
                           [0, 0, 1]])

        extremes = numpy.array(
            [left_back, left_front, right_front, right_back, left_back, frontal_of_robot, right_back]).transpose()

        robot = Hwe.dot(Hec.dot(extremes))
        self.printer.print(robot[0, :], robot[1, :], self.color)
