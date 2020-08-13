#!/usr/bin/env python3

# Copyright (C) 2018 Csaba Jakabos
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

import cv2 as cv
import numpy
# sysv_ipc is needed to access the shared memory where the camera image is present.
import sysv_ipc

# OD4Session is needed to send and receive messages
import OD4Session
# Import the OpenDLV Standard Message Set.
import opendlv_standard_message_set_v0_9_10_pb2
from src.perception import checkIntersection, getKiwiCar, getTrackLines, getConesROI

################################################################################
# This dictionary contains all distance values to be filled by function onDistance(...).
distances = { "front": 0.0, "left": 0.0, "right": 0.0, "rear": 0.0 };

################################################################################
# This callback is triggered whenever there is a new distance reading coming in.
def onDistance(msg, senderStamp, timeStamps):
    #print ("Received distance; senderStamp= %s" % (str(senderStamp)))
    #print ("sent: %s, received: %s, sample time stamps: %s" % (str(timeStamps[0]), str(timeStamps[1]), str(timeStamps[2])))
    #print ("%s" % (msg))
    if senderStamp == 0:
        distances["front"] = msg.distance
        print (msg.distance)
    if senderStamp == 1:
        distances["left"] = msg.distance
    if senderStamp == 2:
        distances["rear"] = msg.distance
    if senderStamp == 3:
        distances["right"] = msg.distance


# Create a session to send and receive messages from a running OD4Session;
# Replay mode: CID = 253
# Live mode: CID = 112
# TODO: Change to CID 112 when this program is used on Kiwi.
session = OD4Session.OD4Session(cid=111)
# Register a handler for a message; the following example is listening
# for messageID 1039 which represents opendlv.proxy.DistanceReading.
# Cf. here: https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L113-L115
messageIDDistanceReading = 1039
session.registerMessageCallback(messageIDDistanceReading, onDistance, opendlv_standard_message_set_v0_9_10_pb2.opendlv_proxy_DistanceReading)
# Connect to the network session.
session.connect()

################################################################################
# The following lines connect to the camera frame that resides in shared memory.
# This name must match with the name used in the h264-decoder-viewer.yml file.
name = "/tmp/img.argb"
# Obtain the keys for the shared memory and semaphores.
keySharedMemory = sysv_ipc.ftok(name, 1, True)
keySemMutex = sysv_ipc.ftok(name, 2, True)
keySemCondition = sysv_ipc.ftok(name, 3, True)
# Instantiate the SharedMemory and Semaphore objects.
shm = sysv_ipc.SharedMemory(keySharedMemory)
mutex = sysv_ipc.Semaphore(keySemCondition)
cond = sysv_ipc.Semaphore(keySemCondition)

################################################################################
# Main loop to process the next image frame coming in.
while True:
    cond.Z()
    mutex.acquire()
    shm.attach()
    buf = shm.read()
    shm.detach()
    mutex.release()

    # Turn buf into img array (1280 * 720 * 4 bytes (ARGB)) to be used with OpenCV.
    img = numpy.frombuffer(buf, numpy.uint8).reshape(720, 1280, 4)

    # Define ROI area and cut it

    ROI = (0, 350, 1280, 370)

    if img.shape[0] == 480:
      ROI = (0, 240, 640, 85)

    img = getConesROI(img, ROI)	

    # Draw white rectangle to cover own kiwi, DO NOT REMOVE
    cv.rectangle(img, (320,370), (960,200), (255, 255, 255),-1)

    # Get tracklines
    cones, farAngle, nearAngle  = getTrackLines(img, True)
    #cv.imshow("cones", cones)

    # Get intersection
    intersection = False
    intersection, farAngleIntersection = checkIntersection(img)

    # Overwrite far and near point with new far point to center of intersection
    if (intersection == True):
        farAngle = farAngleIntersection
        nearAngle = farAngleIntersection

    # Get other kiwi car
    if (intersection == True):
        conesAndCar, kiwiDistanceX, kiwiDistanceY = getKiwiCar(img, True, 0, 0)
    elif (intersection == False):
        conesAndCar, kiwiDistanceX, kiwiDistanceY = getKiwiCar(cones, True, 0, 0)

    #cv.imshow("conesAndCar", conesAndCar)
    #cv.imshow("cones", cones)
    #print("kiwiDistanceX", kiwiDistanceX)
    #print("kiwiDistanceY", kiwiDistanceY)

    # Checking unreasonable high angles
    if (nearAngle > 1.5 or nearAngle < -1.5):
        nearAngle = 0
    if (farAngle > 1.5 or farAngle < -1.5):
        farAngle = 0

    nearPointRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_action_AimDirection()
    farPointRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_action_AimDirection()

    distanceMessageY = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_perception_ObjectDistance()
    distanceMessageY.distance = float(kiwiDistanceY)
    session.send(1134, distanceMessageY.SerializeToString(), senderStamp=0)
    distanceMessageX = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_perception_ObjectDistance()
    distanceMessageX.distance = float(kiwiDistanceX)
    session.send(1134, distanceMessageX.SerializeToString(), senderStamp=1)

    # If it is intersection, take angle to intersection center
    if (intersection == True):
    	nearPointRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_action_AimDirection()
    	nearPointRequest.azimuthAngle = farAngleIntersection
    	session.send(1171, nearPointRequest.SerializeToString(), senderStamp=0)  
  
    	farPointRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_action_AimDirection()
    	farPointRequest.azimuthAngle = farAngleIntersection
    	session.send(1171, farPointRequest.SerializeToString(), senderStamp=1)

    	intersectionMessage = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_perception_Object()
    	intersectionMessage.objectId = int(1)
    	session.send(1130, intersectionMessage.SerializeToString(), senderStamp=1)

    else:
    	nearPointRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_action_AimDirection()
    	nearPointRequest.azimuthAngle = float(nearAngle)
    	session.send(1171, nearPointRequest.SerializeToString(), senderStamp=0)

    	farPointRequest = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_action_AimDirection()
    	farPointRequest.azimuthAngle = float(farAngle)
    	session.send(1171, farPointRequest.SerializeToString(), senderStamp=1)

    	intersectionMessage = opendlv_standard_message_set_v0_9_10_pb2.opendlv_logic_perception_Object()
    	intersectionMessage.objectId = int(0)
    	session.send(1130, intersectionMessage.SerializeToString(), senderStamp=1)
    cv.waitKey(2)
