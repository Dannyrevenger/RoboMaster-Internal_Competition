import cv2 as cv
import numpy as np
import serial
import time

# Change this if you are on different computer
serialcomm = serial.Serial('/dev/tty.usbserial-14140')
serialcomm.baudrate = 9600
serialcomm.bytesize = serial.EIGHTBITS
serialcomm.parity = serial.PARITY_EVEN
serialcomm.stopbits = serial.STOPBITS_ONE
#serialcomm.port = 'COM1'
#serialcomm.open()
camera = "http://admin:admin@192.168.1.2:8081"
#cap = cv.VideoCapture("http://admin:admin@10.79.240.90:8081")
cap = cv.VideoCapture(0)
font = cv.FONT_HERSHEY_COMPLEX
#Setting Camera size
cap.set(cv.CAP_PROP_FRAME_WIDTH, 700)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 700)
#Setting Camera size

#Range Black
lower_black = np.array([0, 0, 0])
higher_black = np.array([124, 110, 148])
#Range Black

#Range Red
lower_red = np.array([155, 25, 0])
higher_red = np.array([179, 255, 255])
bottom_red = np.array([0, 100, 20])
upper_red = np.array([5, 255, 255])

red_u = np.array([0, 186, 0])
red_d = np.array([180, 255, 255])
#Range Red

#Range White
white_low = np.array([0, 0, 156])
white_high = np.array([180, 38, 255])
#Range White
posiiton = ""
count = 0
max_count = 0
while True:
    _, frame = cap.read()
    blurred_frame = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blurred_frame, cv.COLOR_BGR2HSV)  # HSV

    black_mask = cv.inRange(hsv, lower_black, higher_black)  # Blue Mask
    #red_mask_1 = cv.inRange(hsv,lower_red,higher_red)#red_Mask
    #red_mask_2 = cv.inRange(hsv,bottom_red,upper_red)
    #red_mask = red_mask_1+red_mask_2
    red_red = cv.inRange(hsv, red_u, red_d)
    white_mask = cv.inRange(hsv, white_low, white_high)  # Green Mask
    value_white = np.count_nonzero(white_mask)
    value_black = np.count_nonzero(black_mask)
    value_red = np.count_nonzero(red_red)
    value = [value_red, value_black, value_white]
    max_value = value.index(max(value))
    if max_value == 0:
        print('!0######')
        serialcomm.write(b'!0######')
    elif max_value == 1:
        print('!1######')
        serialcomm.write(b'!1######')
    else:
        print('!2######')
        serialcomm.write(b'!2######')
