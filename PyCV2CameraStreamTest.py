import numpy as np
import cv2

# Creat camera 
cap = cv2.VideoCapture(0)

# Thresholds for channel filtering 
smallTh = 240
bigTh   = 255


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Get Individual channels and threshold them 
    blue = frame[:,:,0]
    green = frame[:,:,1]
    red = frame[:,:,2]
    
    retB, blueTh = cv2.threshold(blue, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retG, greenTh = cv2.threshold(green, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retR, redTh = cv2.threshold(red, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    
    # Subtract filtered channels to get true colors
    trueGreen = greenTh - blueTh - redTh
    trueBlue = blueTh - greenTh - redTh
    trueRed = redTh - blueTh - blueTh 


    trueGreen[trueGreen < 0] = 0
    trueBlue[trueBlue < 0]   = 0
    trueRed[trueRed < 0]     = 0
    

    ### Morphological filtering --> find shapes
    # Create shapes
    Gelement = np.ones((20,20))
    Relement = np.ones((8,8))
    
    # filtering
    trueGreen_Opened=cv2.morphologyEx(trueGreen,cv2.MORPH_OPEN,Gelement)
    trueRed_Opened=cv2.morphologyEx(trueRed,cv2.MORPH_OPEN,Relement)
    
    # Display the results
#    cv2.imshow('R',trueRed)
#    cv2.imshow('B',trueBlue)
#    cv2.imshow('G',trueGreen)

    cv2.imshow('Gopened',trueGreen_Opened)
    cv2.imshow('Ropened',trueRed_Opened)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()