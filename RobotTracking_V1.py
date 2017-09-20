import numpy as np
import cv2

# Creat camera 
cap = cv2.VideoCapture(0)

# Thresholds for channel filtering 
smallTh = 250
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
    Gelement1 = np.ones((5,5))
    Relement1 = np.ones((5,5))
    Gelement2 = np.ones((7,7))
    Relement2 = np.ones((7,7))
    Gelement3 = np.ones((11,11))
    Relement3 = np.ones((11,11))
    # filtering
    trueGreen_Closed=cv2.morphologyEx(trueGreen,cv2.MORPH_CLOSE,Gelement1)
    trueRed_Closed=cv2.morphologyEx(trueRed,cv2.MORPH_CLOSE,Relement1)
    trueGreen_Opened=cv2.morphologyEx(trueGreen_Closed,cv2.MORPH_OPEN,Gelement2)
    trueRed_Opened=cv2.morphologyEx(trueRed_Closed,cv2.MORPH_OPEN,Relement2)
    
    
    ### Find the center of our shapes
    # Makes sure only our shape is displayed
    trueGreen_Opened[trueGreen_Opened < 254.5] = 0

    # Get the position of biggest shape
    trueGreenDistance=cv2.distanceTransform(trueGreen_Opened,cv2.DIST_L2,0)
    # print(np.unravel_index(trueGreenDistance.argmax(),trueGreenDistance.shape))
    centre = np.unravel_index(trueGreenDistance.argmax(),trueGreenDistance.shape)
    radius = np.max(trueGreenDistance)
    
    cv2.circle(trueGreenDistance,(centre[1], centre[0]),radius, (255,255,255), 2)

    

    # Display the results
    cv2.imshow('R',trueRed)
#   cv2.imshow('B',trueBlue)
    cv2.imshow('G',trueGreen)


    cv2.imshow('Gopened',trueGreen_Opened)
    cv2.imshow('Ropened',trueRed_Opened)
    cv2.imshow('GdT',trueGreenDistance)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()