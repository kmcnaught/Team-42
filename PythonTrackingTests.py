# -*- coding: utf-8 -*-
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
 

# Start grabbing from camera
camera = cv2.VideoCapture(0)

# Get image size
width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))


# keep looping
while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    
     # show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
     # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
 
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()