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
    
    # Create empty array to display results
    Positions = np.zeros((np.shape(frame)))
    
    # Get Individual channels and threshold them 
    blue = frame[:,:,0]
    green = frame[:,:,1]
    red = frame[:,:,2]
    
    retB, blueTh = cv2.threshold(blue, smallTh, 150, cv2.ADAPTIVE_THRESH_MEAN_C)
    retB, blueThff = cv2.threshold(blue, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retG, greenTh = cv2.threshold(green, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retR, redTh = cv2.threshold(red, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    
    # Do a first morpholical filtering to eliminate noise across channels 
    Element1 = np.ones((15,15))
    Relement1 = np.ones((15,15))
    Gelement2 = np.ones((7,7))
    Relement2 = np.ones((7,7))
    Relement3 = np.ones((45,45))
    Element4 = np.ones((25,25))

    greenDilated=cv2.morphologyEx(greenTh,cv2.MORPH_DILATE,Element1)
    redDilated=cv2.morphologyEx(redTh,cv2.MORPH_DILATE,Element1)
    blueDilated=cv2.morphologyEx(blueThff,cv2.MORPH_DILATE,Element1)
    
    # Subtract filtered channels to get true colors
    trueGreen = greenTh - blueDilated - redDilated
    trueBlue = blueThff - greenDilated - redDilated
    trueRed = redTh - blueDilated     
    trueWhite=np.round(greenTh/3+redTh/3+blueThff/3)

    # Threshold channels correclty
    trueGreen[trueGreen < 0]  = 0
    trueBlue[trueBlue < 0]    = 0
    trueRed[trueRed < 0]      = 0
    trueWhite[trueWhite<250]  = 0
    trueWhite[trueWhite>=250] = 255

    ### Morphological filtering --> find shapes
    # filtering - closing
    trueGreen_Closed=cv2.morphologyEx(trueGreen,cv2.MORPH_CLOSE,Gelement2)
    trueRed_Closed=cv2.morphologyEx(trueRed,cv2.MORPH_CLOSE,Relement2)
    trueBlue_Closed=cv2.morphologyEx(trueBlue,cv2.MORPH_CLOSE,Relement2)
    trueWhite_Closed=cv2.morphologyEx(trueWhite,cv2.MORPH_CLOSE,Relement2)
    # filtering - opening
    trueGreen_Opened=cv2.morphologyEx(trueGreen_Closed,cv2.MORPH_OPEN,Gelement2)
    trueRed_Opened=cv2.morphologyEx(trueRed_Closed,cv2.MORPH_OPEN,Relement2)
    trueBlue_Opened=cv2.morphologyEx(trueBlue_Closed,cv2.MORPH_OPEN,Relement2)
    trueWhite_Opened=cv2.morphologyEx(trueWhite_Closed,cv2.MORPH_OPEN,Relement2)
    trueWhite_To_Hatted=cv2.morphologyEx(trueWhite_Opened,cv2.MORPH_TOPHAT,Relement3)
    trueWhite_Opened=cv2.morphologyEx(trueWhite_To_Hatted,cv2.MORPH_OPEN,Element4)

    ### Find the center of our shapes
    # Makes sure only our shape is displayed
    trueGreen_Opened[trueGreen_Opened < 254.5] = 0
    trueRed_Opened[trueRed_Opened < 254.5] = 0
    trueBlue_Opened[trueBlue_Opened < 254.5] = 0
    trueWhite_Opened[trueWhite_Opened < 254.5] = 0

    ## For Blue channel: find contours and centres 
    cnts = cv2.findContours(trueBlue_Opened.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
         nCnts = np.shape(cnts)[0]
         cnt = cnts[0]
         #cv2.drawContours(Positions,cnts, -1, (255,0,0), 3 )
         for i in np.arange(0, len(cnts)):
             c = cnts[i]
             area = cv2.contourArea(c)
             if area>800:
                 M = cv2.moments(c)
                 center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                 cv2.circle(Positions,center,25, (255,0,0), 2)


    #For White channel: find contours and centres 
    trueWhite_Opened = trueWhite_Opened.astype(np.uint8) #Need to convert to uint8
    cnts = cv2.findContours(trueWhite_Opened.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
         nCnts = np.shape(cnts)[0]
         cnt = cnts[0]
         #cv2.drawContours(Positions,cnts, -1, (255,255,255), 3 )
         for i in np.arange(0, len(cnts)):
             c = cnts[i]
             M = cv2.moments(c)
             center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
             cv2.circle(Positions,center,25, (255,255,255), 2)


    # Get the position of biggest shape
    trueGreenDistance=cv2.distanceTransform(trueGreen_Opened,cv2.DIST_L2,0)
    trueRedDistance=cv2.distanceTransform(trueRed_Opened,cv2.DIST_L2,0)
    
    #get coords
    gcentre = np.unravel_index(trueGreenDistance.argmax(),trueGreenDistance.shape)
    gradius = np.max(trueGreenDistance)
    rcentre = np.unravel_index(trueRedDistance.argmax(),trueRedDistance.shape)
    rradius = np.max(trueRedDistance)
    
    # Draw found centres for reference
    cv2.circle(Positions,(gcentre[1],gcentre[0]),gradius, (0,255,0), 2)
    cv2.circle(Positions,(rcentre[1],rcentre[0]),rradius, (0,0,255), 2)
    
    
    
    #### Display results ##############
    #cv2.imshow('GG',greenTh)
     # Display the results
    #cv2.imshow('R',trueRed)
#    cv2.imshow('b', trueBlue)
#    cv2.imshow('tb', blueTh)
#    cv2.imshow('tbo',trueBlue_Opened)
    #cv2.imshow('G',trueGreen)


    #cv2.imshow('Gopened',trueGreen_Opened)
    #cv2.imshow('Ropened',trueRed_Opened)
    #cv2.imshow('GdT',trueGreenDistance)
    cv2.imshow('W',trueWhite_Opened)
    cv2.imshow('frame',frame)
    cv2.imshow('Position',Positions )

    # Exit if 'q' is pressed 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()