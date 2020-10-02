# Michael Castellano
# EENG 350
# Mini Project
# Continous Aruco Marker Check
# Takes video and checks each frame for potential aruco markers
# Reference: Youtube - "Capture webcam with Python(OpenCV): step by step"

from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
from PIL import Image
from cv2 import aruco
import time
import numpy as np
import argparse
import cv2
import PIL
import math

#print(cv2.__version__)     #3.4.4

global quadrant
quadrant = 0

# Starts recording a video
def cv_exercise0():
    
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    
    cv2.imshow("Image", image)
    
        
# Aruco Marker Test
def cv_exercise5():
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
    if ids != None :   
        cv2.imshow("Image",frame_markers)
        cv_exercise7()


# Angle and Distance calculator
def cv_exercise7():
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
    fov = .12 *1280*7 #3.05mm but in inches
    real_distance = 5.5 # 5.5 inch
    x_distance = (corners[0][0][1][0]- corners[0][0][0][0]) ** 2
    y_distance = (corners[0][0][1][1] - corners[0][0][0][1]) ** 2
    image_distance = corners[0][0][1][0] - corners[0][0][0][0]
    distance = fov * real_distance / image_distance
    
    reference1 = math.sqrt((corners[0][0][3][0] - corners[0][0][0][0])**2 +
                           (corners[0][0][3][1] - corners[0][0][0][1])**2) + corners[0][0][0][0]
                           
    reference2 = math.sqrt((corners[0][0][2][0] - corners[0][0][1][0])**2 +
                           (corners[0][0][2][1] - corners[0][0][1][1])**2) + corners[0][0][2][0]
                           
    imageCenter = (reference1 / 2) + (reference2 / 2) / 2
    print(corners[0][0][0][0])
    print(corners[0][0][0][1])
    x_cord = corners[0][0][0][0]
    y_cord = corners[0][0][0][1]
    # Quad 1
    if ((x_cord < 640.0) & (y_cord < 360.0)):
        global quadrant
        quadrant = 1
    # Quad 2
    elif ((x_cord >= 640.0) & (y_cord < 360.0)):
        global quadrant
        quadrant = 2
    # Quad 3
    elif ((x_cord < 640.0) & (y_cord >= 360.0)):
        global quadrant
        quadrant = 3
    # Quad 4
    elif ((x_cord >= 640.0) & (y_cord >= 360.0)):
        global quadrant
        quadrant = 4
    center = (1280/2) - imageCenter
    angle = math.atan(center / fov) * 180 / 3.14
    
    print("Real Distance: ", distance)
    print("Angle: " , angle)

### Continuously takes photos and checks which quadrant contains an Aruco Marker

# Intilaize Camera
camera = PiCamera(resolution = (1280,720), framerate = 60)
camera.iso =  100
fileName = "marker_test.jpg"

# Begin continous aruco search
print("Press q to stop")
print("Searching for aruco markers")


while True:
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    
    cv2.imshow("Image", image)
    cv_exercise5()
    
# Output 1 - 4 based on where center for marker is located
    # As an int
    print(quadrant)
    
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
