import numpy as np
import numpy.polynomial.polynomial as poly

from .debug import drawPolyline


def fit(imageShape, leftPoint, rightPoints):
    height, width, c = imageShape

    leftCurve = getCurve(imageShape, leftPoint, True)
    rightCurve = getCurve(imageShape, rightPoints, False)
    nearPoint, farPoint = getPoints(imageShape, leftCurve, rightCurve)

    if leftCurve is not None:
        y_new = np.linspace(0, height, 30)
        x_new = poly.polyval(y_new, leftCurve)
        yellowLine = [
            np.array([[xn, yn] for xn, yn in zip(*leftCurve)], dtype=np.int32)
        ]
    else:
        x_new = np.linspace(0, 0, 17)
        y_new = np.linspace(0, height, 17)
        yellowLine = [
            np.array([[xn, yn] for xn, yn in zip(x_new, y_new)], dtype=np.int32)
        ]

    if rightCurve is not None:
        blueLine = [
            np.array([[xn, yn] for xn, yn in zip(*rightCurve)], dtype=np.int32)
        ]
    else:
        x_new = np.linspace(width, width, 17)
        y_new = np.linspace(0, height, 17)
        blueLine = [
            np.array([[xn, yn] for xn, yn in zip(x_new, y_new)], dtype=np.int32)
        ]

    return yellowLine, blueLine, nearPoint, farPoint


def getCurve(imageShape, points, isLeft):
    """
    Fit linear segments the points.
    Adds an extra point to the bottom corner of the image.
    Extrapolations are made by repeating the last value.
    """

    if not any(points):
        return None

    height, width, c = imageShape

    extraWidth = int(width * 0.0 if isLeft else 1.0 * width)
    extra_point = (extraWidth, height)

    sorted_points = sorted([extra_point] + points, key=lambda p: p[1])

    x = np.concatenate([point[::2] for point in sorted_points])
    y = np.concatenate([point[1:2] for point in sorted_points])

    return x, y

def getPoints(imageShape, leftCurve, rightCurve):
    """
    Calculate near and far point from two lines of line segments.
    Averages points from left and right line at specific height.
    If one curve is missing, place the points away from that direction.
    """

    height, width, c = imageShape
    #minY = max(np.min(leftCurve[1]), np.min(rightCurve[1]))
    evalPoints = [0.4 * height, 0.6 * height]

    if leftCurve is None and rightCurve is None:
        return None, None


    nearOffset = 480
    farOffset = 320
 
    if rightCurve is None:
        far_eval = np.interp(evalPoints[0], leftCurve[1], leftCurve[0])
        near_eval = np.interp(evalPoints[1], leftCurve[1], leftCurve[0])
        farPoint = (int(far_eval) + farOffset, int(evalPoints[0]))
        nearPoint = (int(near_eval) + nearOffset, int(evalPoints[1]))
        return nearPoint, farPoint
 
    if leftCurve is None:
        far_eval = np.interp(evalPoints[0], rightCurve[1], rightCurve[0])
        near_eval = np.interp(evalPoints[1], rightCurve[1], rightCurve[0])
        farPoint = (int(far_eval) - farOffset, int(evalPoints[0]))
        nearPoint = (int(near_eval) - nearOffset, int(evalPoints[1]))
        return nearPoint, farPoint

    farPoint = (
        int(
            (np.interp(evalPoints[0], leftCurve[1], leftCurve[0])
            + np.interp(evalPoints[0], rightCurve[1], rightCurve[0])) / 2
        ),
        int(evalPoints[0]),
    )
    nearPoint = (
        int(
            (np.interp(evalPoints[1], leftCurve[1], leftCurve[0])
            + np.interp(evalPoints[1], rightCurve[1], rightCurve[0])) / 2
        ),
        int(evalPoints[1]),
    )

    return nearPoint, farPoint
