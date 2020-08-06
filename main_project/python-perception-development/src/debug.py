import cv2 as cv
import numpy as np
import numpy.polynomial.polynomial as poly


def drawBoundingBox(image, boundingBoxes, color):
    for (x, y, w, h) in boundingBoxes:
        p1 = (x, y)
        p2 = (x + w, y + h)
        cv.rectangle(image, p1, p2, color, 2)


def drawBlackPolylineLeft(image, points, color):
	if len(points) >= 4:
		points.pop()
	cv.polylines(
	image, [np.array(points[::2], dtype=np.int32)], False, color, 6,
	)

def drawBlackPolylineRight(image, points, color):
	if len(points) >= 3:
		del points[2]
	cv.polylines(
	image, [np.array(points, dtype=np.int32)], False, color, 6,
	)

def drawPolyline(image, points, color):
    cv.polylines(
        image, [np.array(points, dtype=np.int32)], False, color, 6,
    )
