import cv2 as cv
import numpy as np

import src.linear_segments as linear_segments
import src.polynomial as polynomial

from .debug import drawBoundingBox, drawPolyline

hsvColorRanges = {
    "blue": [(110, 50, 50), (130, 255, 255)],
    "yellow": [(15, 0, 0), (36, 255, 255)],
}


def getConesROI(image, ROI):
    x, y, w, h = ROI
    return image[y : y + h, x : x + w]


def simplifyMask(mask):
    iterations = 4
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    mask = cv.dilate(mask, kernel, iterations=iterations)
    mask = cv.erode(mask, kernel, iterations=iterations)
    return mask


def filterBoundingBoxRightLane(boundingBox):
    x, y, width, height = boundingBox
    isTooLarge = height > 200 or width > 70
    isTooSmall = height < 15 or width < 3
    isTooLeft = y < 180 and x < 320
    isTooRight = y < 80 and x > 960

    aspectRatio = width / height
    isTooOddlyShaped = aspectRatio > 1.5 or (1.0 / aspectRatio) > 3

    return not (isTooLarge or isTooSmall or isTooOddlyShaped or isTooLeft or isTooRight)

def filterBoundingBoxLeftLane(boundingBox):
    x, y, width, height = boundingBox
    isTooLarge = height > 250 or width > 100
    isTooSmall = height < 15 or width < 3
    isTooRight = y < 180 and x >= 960
    isTooLeft = y < 80 and x < 320

    aspectRatio = width / height
    isTooOddlyShaped = aspectRatio > 1 or (1.0 / aspectRatio) > 3

    return not (isTooLarge or isTooSmall or isTooOddlyShaped or isTooRight or isTooLeft)

def getBoundingBoxCenter(boundingBox):
    x, y, w, h = boundingBox
    return (x + w / 2, y + h / 2)

def checkIntersection(image):
    [height, width, c] = image.shape
    # add 2 shapes to check only cone area for red colors

    # the width of shape is defined by the 20 amd 120 values,
    # increase their distance to grow area, eg. 20->30 and 120->150 at pt1,2,3,4
    # Shape 1
    canvas_red = image.copy()
    canvas_red_mask = np.zeros(canvas_red.shape[:2], dtype=canvas_red.dtype)
    pt1 = (int(width/2.1) - 20, 25)
    pt2 = (int(width/2.1) - 400,25)
    pt3 = (0 + 0, height)
    pt4 = (0 + 160, height)
    triangle_cnt = np.array( [pt1, pt2, pt3, pt4] )
    cv.drawContours(canvas_red_mask, [triangle_cnt], 0, (255, 255, 255), -1)

    # Shape 2
    pt1 = (width-int(width/2.1) + 20, 25)
    pt2 = (width-int(width/2.1) + 400, 25)
    pt3 = (width - 0, height)
    pt4 = (width - 160, height)
    triangle_cnt = np.array( [pt1, pt2, pt3, pt4] )
    cv.drawContours(canvas_red_mask, [triangle_cnt], 0, (255, 255, 255), -1)

    # keep only shaped area and check for red cones
    canvas_red = cv.bitwise_and(canvas_red,canvas_red, mask= canvas_red_mask)

    redConehsvImage = cv.cvtColor(canvas_red, cv.COLOR_BGR2HSV)
    redConeMask = cv.inRange(redConehsvImage, (150, 100, 20), (180, 255, 255))
    redConeMask = cv.morphologyEx(redConeMask, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_ELLIPSE, (9,9)));
    a, redConesList,b  = cv.findContours(redConeMask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Draw rectangle around red cones to get center
    x, y, w, h = cv.boundingRect(a)
    farPoint = (int(x + w/2),int(y+h/2))
    cv.circle(canvas_red, farPoint, 5, (160, 206, 165) , 2)

    # calculate far point based on center
    fovy = 48.8
    fovx = (width / height) * fovy
    farAngle = np.deg2rad(((farPoint[0] - width / 2) / width) * fovx)								   


    # Count how many red cones per side
    counter1 = 0
    counter2 = 0
    for i in range(len(redConesList)):
        x, y, w, h = cv.boundingRect(redConesList[i])
        #print(x)
        if x < 1280/2:
                counter1 = counter1 +1
        if x > 1280/2:
                counter2 = counter2 +1

    # If more than 2 red cones each side set intersection True
    intersection = False

    if (counter1 > 1 and counter2 > 2) or (counter1 > 2 and counter2 > 1) :
        intersection = True

    return intersection, farAngle


def getTrackLines(image, halt):
    """Data is kept in a
    {
        "color": value,
        ...
    }
    format in order to simplify the code.
    """
    # Get image dimensions
    [height, width, c] = image.shape

    # blur and hsv
    blurredImage = cv.GaussianBlur(image, (11, 11), 2)
    hsvImage = cv.cvtColor(blurredImage, cv.COLOR_BGR2HSV)

    # Select only the regions with specified color
    coneImages = {
        color: cv.inRange(hsvImage, ranges[0], ranges[1])
        for color, ranges in hsvColorRanges.items()
    }

    # Turn selected regions into grayscale (binary really)
    binaryConeImages = {
        color: cv.threshold(coneImage, 0, 255, cv.THRESH_BINARY)[1]
        for color, coneImage in coneImages.items()
    }

    # Simplify masks (the binary images)
    simplifiedBinaryConeImages = {
        color: simplifyMask(mask) for color, mask in binaryConeImages.items()
    }

    # Find all contours in the binary images
    coneContours = {
        color: cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        for color, img in simplifiedBinaryConeImages.items()
    }

    # Get boundingboxes
    coneBoundingBoxes = {
        color: [cv.boundingRect(contour) for contour in contours]
        for color, contours in coneContours.items()
    }

    # Remove boundingboxes based on criteria
    filteredConeBoundingBoxesRightLane = {
        color: list(filter(filterBoundingBoxRightLane, boundingBoxes))
        for color, boundingBoxes in coneBoundingBoxes.items()
    }

    # Remove boundingboxes based on criteria
    filteredConeBoundingBoxesLeftLane = {
        color: list(filter(filterBoundingBoxLeftLane, boundingBoxes))
        for color, boundingBoxes in coneBoundingBoxes.items()
    }

    canvas = image.copy()

    yellowLine, blueLine, nearPoint, farPoint = linear_segments.fit(
    #yellowLine, blueLine, nearPoint, farPoint = polynomial.fit(
        image.shape,
        list(map(getBoundingBoxCenter, filteredConeBoundingBoxesLeftLane["yellow"])),
        list(map(getBoundingBoxCenter, filteredConeBoundingBoxesRightLane["blue"]))
    )

    # Flipping order of yellow line to make fillPoly
    yellowLine = np.flip(yellowLine, 1)

    # Fill up area between lines
    pts = np.hstack((blueLine, yellowLine))

    # blank mask
    # the mask has same height and width as `canvas`, but only 1 color channel
    mask = np.zeros(canvas.shape[:-1]).astype(np.uint8)

    # our mask - some `mask_value` contours on black (zeros) background,
    fill_color = [0, 255, 0]
    mask_value = 255

    # fill up area between lines with mask_value
    cv.fillPoly(mask, np.int_([pts]), mask_value)

    # select all that is not mask_value
    sel = mask != mask_value

    # and fill it with fill_color
    canvas[sel] = 255

    # Calculate near and far angle
    fovy = 48.8
    fovx = (width / height) * fovy

    if farPoint is not None:
        farAngle = np.deg2rad(((farPoint[0] - width / 2) / width) * fovx)
    else:
        farAngle = 0.0

    if nearPoint is not None:
        nearAngle = np.deg2rad(((nearPoint[0] - width / 2) / width) * fovx)
    else:
        nearAngle = 0.0

    ################## DEBUG below ##################

    drawPolyline(canvas, yellowLine, (0, 255, 0))
    drawPolyline(canvas, blueLine, (0, 255, 0))
    drawBoundingBox(canvas, filteredConeBoundingBoxesRightLane["blue"], (94, 206, 165))
    drawBoundingBox(canvas, filteredConeBoundingBoxesLeftLane["yellow"], (94, 206, 165))

    # Adding far point
    cv.circle(canvas, farPoint, 5, (255, 255, 255) , 2)
    cv.circle(canvas, nearPoint, 5, (94, 206, 165) , 2)

    return canvas, farAngle, nearAngle

def getKiwiCar(image, halt, cutPosX, cutPosY):

    # Crop, blur and hsv
    blurredImage = cv.GaussianBlur(image, (11, 11), 2)
    hsvImage = cv.cvtColor(blurredImage, cv.COLOR_BGR2HSV)

    # Red mask, lower and upper, as they span around 180
    maskRed1 = cv.inRange(hsvImage, (0, 100, 50), (10, 255, 255))
    maskRed2 = cv.inRange(hsvImage, (150, 100, 50), (180, 255, 255))
    maskRed = cv.bitwise_or(maskRed1, maskRed2)

    # Black mask
    maskBlack = cv.inRange(hsvImage, (0, 0, 0), (180, 255, 30))


    # Combined mask
    mask = cv.bitwise_or(maskRed, maskBlack)

    # Threshold, erode and dilate
    mask = simplifyMask(mask)


    # Combine near areas
    thresh_gray = cv.morphologyEx(mask, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_ELLIPSE, (15,15)));

    # Find contours
    imageContour, kiwiContours, hierarchy = cv.findContours(thresh_gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    ################## DEBUG below ##################


    # Create canvas
    canvas2 = image.copy()
    [height, width, c] = canvas2.shape

    # Set unreasonable large distance
    kiwiDistanceX = 6666
    kiwiDistanceY = 6666


    # Check biggest contour and draw rectangle around it
    if (len(kiwiContours) != 0):
        # find the biggest countour (c) by the area
        c = max(kiwiContours, key = cv.contourArea)
        x, y, w, h = cv.boundingRect(c)

        # draw the biggest contour (c) in green and size filter it
        if (h > 30 and w > 60 and w / h > 1.5):

            kiwiDistanceX = cutPosX + x - 1280/2
            kiwiDistanceY = 720 - cutPosY - h


            cv.rectangle(image,(x,y),(x+w,y+h),(0,255,0),-1)


    return image, kiwiDistanceX, kiwiDistanceY
