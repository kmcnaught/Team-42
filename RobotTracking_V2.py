import numpy as np
import cv2

# Creat camera 
cap = cv2.VideoCapture(0)

# Thresholds for channel filtering 
smallTh = 230
bigTh   = 255


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Get Individual channels and threshold them 
    blue = frame[:,:,0]
    green = frame[:,:,1]
    red = frame[:,:,2]
    
    retB, blueTh = cv2.threshold(blue, 230, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retG, greenTh = cv2.threshold(green, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retR, redTh = cv2.threshold(red, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    
    Element1 = np.ones((15,15))
    Relement1 = np.ones((15,15))
    Gelement2 = np.ones((7,7))
    Relement2 = np.ones((7,7))
    greenDilated=cv2.morphologyEx(greenTh,cv2.MORPH_DILATE,Element1)
    redDilated=cv2.morphologyEx(redTh,cv2.MORPH_DILATE,Element1)
    blueDilated=cv2.morphologyEx(blueTh,cv2.MORPH_DILATE,Element1)
    # Subtract filtered channels to get true colors
    trueGreen = greenTh - blueDilated - redDilated
    trueBlue = blueTh - greenDilated - redDilated
    trueRed = redTh - blueDilated     
    trueWhite=(greenTh/3+redTh/3+blueTh/3)

    trueGreen[trueGreen < 0] = 0
    trueBlue[trueBlue < 0]   = 0
    trueRed[trueRed < 0]     = 0
    trueWhite[trueWhite<250] = 0
    trueWhite[trueWhite>=250] = 255

    ### Morphological filtering --> find shapes
    # Create shapes


    # filtering
    trueGreen_Closed=cv2.morphologyEx(trueGreen,cv2.MORPH_CLOSE,Gelement2)
    trueRed_Closed=cv2.morphologyEx(trueRed,cv2.MORPH_CLOSE,Relement2)
    trueBlue_Closed=cv2.morphologyEx(trueBlue,cv2.MORPH_CLOSE,Relement2)
    trueGreen_Opened=cv2.morphologyEx(trueGreen_Closed,cv2.MORPH_OPEN,Gelement2)
    trueRed_Opened=cv2.morphologyEx(trueRed_Closed,cv2.MORPH_OPEN,Relement2)
    trueBlue_Opened=cv2.morphologyEx(trueBlue_Closed,cv2.MORPH_OPEN,Relement2)

    
    ### Find the center of our shapes
    # Makes sure only our shape is displayed
    trueGreen_Opened[trueGreen_Opened < 254.5] = 0
    trueRed_Opened[trueRed_Opened < 254.5] = 0
    trueBlue_Opened[trueBlue_Opened < 254.5] = 0

    # Get the position of biggest shape
    trueGreenDistance=cv2.distanceTransform(trueGreen_Opened,cv2.DIST_L2,0)
    trueRedDistance=cv2.distanceTransform(trueRed_Opened,cv2.DIST_L2,0)
    
    #get coords
    gcentre = np.unravel_index(trueGreenDistance.argmax(),trueGreenDistance.shape)
    gradius = np.max(trueGreenDistance)
    rcentre = np.unravel_index(trueRedDistance.argmax(),trueRedDistance.shape)
    rradius = np.max(trueRedDistance)
    
    # Draw found centres for reference
    Positions = np.zeros((np.shape(frame)))
    cv2.circle(Positions,(gcentre[1],gcentre[0]),gradius, (0,255,0), 2)
    cv2.circle(Positions,(rcentre[1],rcentre[0]),rradius, (0,0,255), 2)
    
    
    
    #### Display results
#    cv2.imshow('GG',greenTh)

#    # Display the results
#    cv2.imshow('R',trueRed)
    #cv2.imshow('tB',trueBlue_Opened)
#    cv2.imshow('G',trueGreen)

    cv2.imshow('frame',frame)
    cv2.imshow('Gopened',greenTh)
#    cv2.imshow('Ropened',trueRed_Opened)
#    cv2.imshow('GdT',trueGreenDistance)
    cv2.imshow('W',trueWhite)

    
    cv2.imshow('Position',Positions )

    # Exit if 'q' is pressed 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()