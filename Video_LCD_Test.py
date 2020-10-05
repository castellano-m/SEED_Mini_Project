# Michael Castellano
# EENG 350
# Mini Project
# Continous Aruco Marker Check
# Takes video and checks each frame for potential aruco markers

### GLARE FIX!!!!!####
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
import serial
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
#Set address
arduino = serial.Serial('/dev/ttyACM0', 115200)
#Wait for connection to complete
time.sleep(3)

# Set up the LCD screen
lcd_columns = 16
lcd_rows = 2
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [100, 100, 100] # White screen

# Gloval Variables
global position, oldVal, quadrant, aruco_dict, parameters, frame, corners, ids, rejectedImgPoints, frame_markers
quadrant = '0'
oldVal = '0'
# Turn on video
video = cv2.VideoCapture(0)
        
def aruco_location():
#    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
#    parameters = aruco.DetectorParameters_create()
#    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
#    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
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
    x_cord = corners[0][0][0][0]
    y_cord = corners[0][0][0][1]
    # Quad 1
    if ((x_cord < 320.0) and (y_cord < 180.0)):
        quadrant = '1'
    # Quad 2
    elif ((x_cord >= 320.0) and (y_cord < 180.0)):
        quadrant = '2'
    # Quad 3
    elif ((x_cord < 320.0) and (y_cord >= 180.0)):
        quadrant = '3'
    # Quad 4
    elif ((x_cord >= 320.0) and (y_cord >= 180.0)):
        quadrant = '4'
    center = (1280/2) - imageCenter
    angle = math.atan(center / fov) * 180 / 3.14

    if quadrant != oldVal:
        print(oldVal)
        position = arduino.readline().decode('utf-8').rstrip()
        lcd.message = "Setpoint: %c\nPosition: %s" %(quadrant, position)
        arduino.write(quadrant.encode())
        global oldVal
        oldVal = quadrant
        print(oldVal)

    
# MAIN
while True:
    # Get current frame and convert to grayscale. Print frame
    check,frame = video.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Capturing", frame)
    
    # Checks for aruco marker
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
    if ids != None :   
        cv2.imshow("Capturing",frame_markers)
        aruco_location()
        
    else:
        quadrant = '0'
            

# Stop recoring if 'q' is pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    
# Stop video and close all windows
video.release()
cv2.destroyAllWindows()
lcd.clear()