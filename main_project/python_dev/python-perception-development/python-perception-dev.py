import os
import sys

import cv2 as cv

from src.perception import checkIntersection, getKiwiCar, getTrackLines

assert (
    len(sys.argv) == 2 and sys.argv[1] is not None
), "You need to enter the path to a image in order to run this program"

input_path = sys.argv[1]

#handling full folders
def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)

    return images

#read full folder if folder is given
if os.path.isdir(input_path):
    print('Reading full folder')
    images = load_images_from_folder(input_path)
    lenImages = len(images)

    for i in range(len(images)):
        image = images[i]
        # ROI for emergency braking
        kiwiROI = (320, 150, 640, 720-300)
        x, y, w, h = kiwiROI
        kiwiROIArea = image[y : y + h, x : x + w]
        #cv.imshow("kiwiROIArea", kiwiROIArea)

        # Get intersection
        intersection = checkIntersection(image)

        cones, farAngle, nearAngle = getTrackLines(image, True)
        #cv.imshow("cones", cones)

        # Get Kiwi car, if it is trimmed picture send trim position to be able
        # to offset kiwi measurement
        # Sending starting X, Y position of the ROI area, 0 for not cutted picture
        if (intersection == True):
            car, kiwiDistanceX, kiwiDistanceY = getKiwiCar(image, True, 0, 0)
        elif (intersection == False):
            car, kiwiDistanceX, kiwiDistanceY = getKiwiCar(kiwiROIArea, True, 320, 150)
        cv.imshow("car", car)
        cv.waitKey(0)

#read file only if file is given
elif os.path.isfile(input_path):
    print('Reading single file')
    image = cv.imread(input_path)
    # ROI for emergency braking
    kiwiROI = (320, 150, 640, 720-300)
    x, y, w, h = kiwiROI
    kiwiROIArea = image[y : y + h, x : x + w]
    #cv.imshow("kiwiROIArea", kiwiROIArea)
    cv.imshow("image", image)


    cv.rectangle(image, (320,720), (960,550), (255, 255, 255),-1)
    car, kiwiDistanceX, kiwiDistanceY = getKiwiCar(image, True, 0, 0)
    #cv.imshow("carrrr", car)
    # Get intersection



    # If no intersection get the track lines

    cones, farAngle, nearAngle = getTrackLines(image, True)
    conesAndCar, kiwiDistanceX, kiwiDistanceY = getKiwiCar(cones, True, 0, 0)
    intersection = checkIntersection(conesAndCar)
    cv.imshow("conesAndCar", conesAndCar)
    print("farAngle", farAngle)
    print("nearAngle", nearAngle)
    # Get Kiwi car, if it is trimmed picture send trim position to be able
    # to offset kiwi measurement
    # Sending starting X, Y position of the ROI area, 0 for not cutted picture
    if (intersection == True):
        car, kiwiDistanceX, kiwiDistanceY = getKiwiCar(image, True, 0, 0)
    elif (intersection == False):
        car, kiwiDistanceX, kiwiDistanceY = getKiwiCar(kiwiROIArea, True, 320, 150)

    #cv.imshow("car", car)
    cv.waitKey(0)
