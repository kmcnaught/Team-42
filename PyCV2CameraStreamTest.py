import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Get Individual channels and threshold them 
    
    blue = frame[:,:,0]
    green = frame[:,:,1]
    red = frame[:,:,2]
    
    retB, blueTh = cv2.threshold(blue, 230, 255, cv2.ADAPTIVE_THRESH_MEAN_C)
    retG, greenTh = cv2.threshold(green, 230, 255, cv2.ADAPTIVE_THRESH_MEAN_C)
    retR, redTh = cv2.threshold(red, 230, 255, cv2.ADAPTIVE_THRESH_MEAN_C)
    
    trueGreen = greenTh - blueTh - redTh
    trueGreen[trueGreen < 0]=0
    
    
    
    # Display the resulting frame
    ###cv2.imshow('G',(frame[:,:,1]-frame[:,:,0])-frame[:,:,2])
    cv2.imshow('R',redTh)
    cv2.imshow('B',blueTh)
    cv2.imshow('G',greenTh)
    cv2.imshow('tG',trueGreen)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()