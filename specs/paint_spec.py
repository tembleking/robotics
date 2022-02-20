from doublex import Spy, called, assert_that
from mamba import description, it

from robotics.paint import RobotPainter, Size, Color
from specs import np_close

with description('Paint') as self:
    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([0, 0, 0])

        assert_that(spy.print, called().with_args(np_close([-0.09, 0.11, 0.11, -0.09, -0.09, 0.11, -0.09]),
                                                  np_close([0.05, 0.05, -0.05, -0.05, 0.05, 0.0, -0.05]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([1, 0, 0])

        assert_that(spy.print, called().with_args(np_close([0.91, 1.11, 1.11, 0.91, 0.91, 1.11, 0.91]),
                                                  np_close([0.05, 0.05, -0.05, -0.05, 0.05, 0.0, -0.05]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([0, 1, 0])

        assert_that(spy.print, called().with_args(np_close([-0.09, 0.11, 0.11, -0.09, -0.09, 0.11, -0.09]),
                                                  np_close([1.05, 1.05, 0.95, 0.95, 1.05, 1., 0.95]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([0, 0, 1])

        assert_that(spy.print, called().with_args(
            np_close([-0.09070076, 0.0173597, 0.1015068, -0.00655366, -0.09070076, 0.05943325, -0.00655366]),
            np_close([-0.04871727, 0.11957692, 0.06554669, -0.1027475, -0.04871727, 0.09256181, -0.1027475]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy, color=Color.OTHER)
        painter.paint([0, 0, 0])

        assert_that(spy.print, called().with_args(np_close([-0.09, 0.11, 0.11, -0.09, -0.09, 0.11, -0.09]),
                                                  np_close([0.05, 0.05, -0.05, -0.05, 0.05, 0.0, -0.05]), 1))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy, size=Size.BIG)
        painter.paint([0, 0, 0])

        assert_that(spy.print, called().with_args(np_close([-0.45, 0.55, 0.55, -0.45, -0.45, 0.55, -0.45]),
                                                  np_close([0.25, 0.25, -0.25, -0.25, 0.25, 0., -0.25]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([1, 1, 0])

        assert_that(spy.print, called().with_args(np_close([0.91, 1.11, 1.11, 0.91, 0.91, 1.11, 0.91]),
                                                  np_close([1.05, 1.05, 0.95, 0.95, 1.05, 1., 0.95]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([1, 0, 1])

        assert_that(spy.print,
                    called().with_args(
                        np_close([0.90929924, 1.0173597, 1.1015068, 0.99344634, 0.90929924, 1.05943325, 0.99344634]),
                        np_close(
                            [-0.04871727, 0.11957692, 0.06554669, -0.1027475, -0.04871727, 0.09256181, -0.1027475]), 0))

    with it('paints the localization of the robot'):
        spy = Spy()
        painter = RobotPainter(printer=spy)
        painter.paint([0, 1, 1])

        assert_that(spy.print, called().with_args(
            np_close([-0.09070076, 0.0173597, 0.1015068, -0.00655366, -0.09070076, 0.05943325, -0.00655366]),
            np_close([0.95128273, 1.11957692, 1.06554669, 0.8972525, 0.95128273, 1.09256181, 0.8972525]), 0))
