import numpy as np
import numpy.polynomial.polynomial as poly

from .debug import drawPolyline


def fit(imageShape, leftPoint, rightPoints):
    height, width, c = imageShape

    left_coeffs  = getCurve(imageShape, leftPoint, True)
    rigth_coeffs = getCurve(imageShape, rightPoints, False)
    nearPoint, farPoint = getPoints(imageShape, left_coeffs, rigth_coeffs)

    if left_coeffs is not None:
        y_new = np.linspace(0, height, 15)
        x_new = poly.polyval(y_new, left_coeffs)
        yellowLine = [np.array([[xn,yn] for xn,yn in zip(x_new,y_new)],dtype=np.int32)]

    else:
        x_new = np.linspace(0, 0, 17)
        y_new = np.linspace(0, height, 17)
        yellowLine = [np.array([[xn,yn] for xn,yn in zip(x_new,y_new)],dtype=np.int32)]

    if rigth_coeffs is not None:
        y_new = np.linspace(0, height, 15)
        x_new = poly.polyval(y_new, rigth_coeffs)
        blueLine = [np.array([[xn,yn] for xn,yn in zip(x_new,y_new)],dtype=np.int32)]
    else:
        x_new = np.linspace(width, width, 17)
        y_new = np.linspace(0, height, 17)
        blueLine = [np.array([[xn,yn] for xn,yn in zip(x_new,y_new)],dtype=np.int32)]

    return yellowLine, blueLine, nearPoint, farPoint


def getCurve(imageShape, points, isLeft):
    """
    Fit polynomial curve to the points.
    Adds an extra point to the bottom corner of the image.
    """

    if not any(points):
        return None

    height, width, c = imageShape

    x = np.concatenate([point[::2] for point in points])
    y = np.concatenate([point[1:2] for point in points])

    y = np.insert(
        y, 0, height
    )  # Create one point were the image starts and one behind the car
    if isLeft:  # Is it possible to check which side the cones are before we start?
        x = np.insert(x, 0, width * 0.1)
    else:
        x = np.insert(x, 0, width * 0.9)

    if len(x) < 4:  # Different degree depending on the number of cones
        coefs = poly.polyfit(y, x, 1)
    else:
        coefs = poly.polyfit(y, x, 2)
    return coefs


def getPoints(imageShape, left_coeffs, right_coeffs):
    """
    Evaluate two polynomial curves at two separate points
    and average their height in order to get a near and far point.
    If one curve is missing, place the points away from that direction.
    """

    height, width, c = imageShape
    eval_points = [0.3 * height, 0.8 * height]

    if left_coeffs is None and right_coeffs is None:
        return None, None

    if right_coeffs is None:
        left_evals = poly.polyval(eval_points, left_coeffs)
        far_point = (int(left_evals[0]) + 100, int(eval_points[0]))
        near_point = (int(width / 2) + 100, int(eval_points[1]))
        return near_point, far_point

    if left_coeffs is None:
        right_evals = poly.polyval(eval_points, right_coeffs)
        far_point = (int(right_evals[0]) - 100, int(eval_points[0]))
        near_point = (int(width / 2) - 100, int(eval_points[1]))
        return near_point, far_point

    right_evals = poly.polyval(eval_points, left_coeffs)
    left_evals = poly.polyval(eval_points, right_coeffs)

    near_point = (int((right_evals[1] + left_evals[1]) / 2), int(eval_points[1]))
    far_point = (int((right_evals[0] + left_evals[0]) / 2), int(eval_points[0]))

    return near_point, far_point
