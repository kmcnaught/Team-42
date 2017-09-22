### INTRO GOES HERE ####
########################

#### IMPORTS ####
import numpy as np
import cv2
import math
import cmath
import serial

#### SET UP ####
# Creat camera 
cap = cv2.VideoCapture(0)

# Create serial to arduino
ser = serial.Serial('COM6', 9600, timeout=0.1)

# Thresholds for behaviour control
TgtArea          = 1200 # Min area of a baloon 
TgtDistThres     = 100 # Min dist from tgt before entering killing mode
RobArea          = 5 # Min area of robots lables

# Flags
GreenCheck, RedCheck, WhiteCheck, TgtCheck, AlignCheck, BlueCheck = 0, 0, 0, 0, 0, 0 # Equal 0 if we don't have a tgt
command = 0
TgtCentre = [0,0]
TgtAngle = 0; 
TgtDist  = 4;

## Functions
def TgtIdentif(gcentre, rcentre, WhiteList, Positions):

    ## Find closest white target
    robVec= [gcentre[1]-rcentre[1],gcentre[0]-rcentre[0]]
    WhiteDists = np.zeros((len(WhiteList),1))
    for i in np.arange(0, len(WhiteList)):
        whiteVec = [WhiteList[i,1]-gcentre[1],WhiteList[i,0]-gcentre[0]]
        WhiteDists[i] =  whiteVec[0]**2 + whiteVec[1]**2
    WhiteDists = WhiteDists.astype(np.int32)
    MinDist = np.argmin(WhiteDists)
    TgtDist = WhiteDists[MinDist]
    TgtCentre = WhiteList[MinDist,:]
    TgtCentre = TgtCentre.astype(np.int16)
    
    ## Find angle to target
    tgtVec = [TgtCentre[1]-gcentre[1], TgtCentre[0]-gcentre[0]]
    TgtAngle=cmath.phase(complex(tgtVec[1],tgtVec[0]))-cmath.phase(complex(robVec[1],robVec[0]))

    #TgtAngle = angle_between(robVec, tgtVec)  # Angle in radian
    TgtAngle = TgtAngle * 180 / math.pi       # Angle in degrees
    TgtAngle = (( TgtAngle +180 ) % 360) - 180  # Remap to values between -179 and +180
    TgtCheck = 1

    return WhiteDists, TgtCentre, TgtCheck, TgtAngle, TgtDist



def RobAligntoTgt(gcentre,TgtCentre, TgtAngle, TgtDist):  # !!! Align function should check if pathsis clear as well
    AngleTh = 15 

    if math.fabs(int(round(TgtAngle))) < AngleTh:
        # We are aligned 
        ## Adding here a check for distance from target: if we are safely away we shouldn't be in killing mode
        print('Tgt Dist', TgtDist, type(TgtDist))
        if math.sqrt(TgtDist) < TgtDistThres:
            print('Go Kill')
            command = 0
        else:
            command = 2
            print('Go Forward... safely')
        AlignCheck = 1
    else: 
        if int(round(TgtAngle)) < 0:
            # Turn right
            command = 3
            print('Turn Right')

        else:
            # Turn left
            command = 4
            print('Turn Left')
        AlignCheck = 0
    return AlignCheck, command

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return math.acos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))    


def Vision(frame, TgtCentre, TgtCheck, TgtAngle, AlignCheck, TgtDistThres, command, TgtDist,TgtArea): 

    ## Setting up ##
    # Create empty array to display results
    Positions = np.zeros((np.shape(frame)))
    cv2.imshow('frame', frame)
    
    # Thresholds for  HSV filtering 
    WhiteTh  = 60
    WhiteTh2 = 15
    BlackTh = 150
    lGTh    = 35
    hGTh    = 75
    lRTh    = 170
    hRTh    = 5
    lBTh    = 80
    hBTh    = 135

    ## HSV filtering 
    # Convert to HSV
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Threshold for each color
    greenTh = np.greater(np.greater(frame[:,:,0],lGTh)*1 + np.less(frame[:,:,0],hGTh)*1 + np.greater(frame[:,:,1],WhiteTh)*1 + np.greater(frame[:,:,2],BlackTh)*1,3) 
    greenTh = greenTh*255
    trueGreen = greenTh.astype(np.uint8)
    
    redTh = np.greater(np.greater(frame[:,:,0],lRTh)*1 + np.less(frame[:,:,0],hRTh)*1 + np.greater(frame[:,:,1],WhiteTh)*1 + np.greater(frame[:,:,2],BlackTh)*1,2) 
    redTh = redTh*255
    trueRed = redTh.astype(np.uint8)

    blueTh = np.greater(np.greater(frame[:,:,0],lBTh)*1 + np.less(frame[:,:,0],hBTh)*1 + np.greater(frame[:,:,1],WhiteTh)*1 + np.greater(frame[:,:,2],BlackTh)*1,3) 
    blueTh = blueTh*255
    trueBlue = blueTh.astype(np.uint8)
    
    whiteTh = np.greater(np.less(frame[:,:,1],WhiteTh2)*1 + np.greater(frame[:,:,2],BlackTh)*1,1) 
    whiteTh = whiteTh*255
    trueWhite = whiteTh.astype(np.uint8)

    # Morphological filtering: select only things of approx right size
    Element4  = np.ones((15,15))
    Gelement2 = np.ones((7,7))
    Relement2 = np.ones((7,7))
    Relement3 = np.ones((35,35))

    # Morphological filtering - closing
    trueGreen_Closed    = cv2.morphologyEx(trueGreen,cv2.MORPH_CLOSE,Gelement2)
    trueRed_Closed      = cv2.morphologyEx(trueRed,cv2.MORPH_CLOSE,Relement2)
    trueBlue_Closed     = cv2.morphologyEx(trueBlue,cv2.MORPH_CLOSE,Relement2)
    trueWhite_Closed    = cv2.morphologyEx(trueWhite,cv2.MORPH_CLOSE,Relement2)

    # Morphological filtering - opening
    trueGreen_Opened    = cv2.morphologyEx(trueGreen_Closed,cv2.MORPH_OPEN,Gelement2)
    trueRed_Opened      = cv2.morphologyEx(trueRed_Closed,cv2.MORPH_OPEN,Relement2)
    trueBlue_Opened     = cv2.morphologyEx(trueBlue_Closed,cv2.MORPH_OPEN,Relement2)
    trueWhite_Opened    = cv2.morphologyEx(trueWhite_Closed,cv2.MORPH_OPEN,Relement2)
    trueWhite_To_Hatted = cv2.morphologyEx(trueWhite_Opened,cv2.MORPH_TOPHAT,Relement3)
    trueWhite_Opened    = cv2.morphologyEx(trueWhite_To_Hatted,cv2.MORPH_OPEN,Element4)

    ## Finding Centres of shapes ##
    # Makes sure only our shape is displayed
#    trueGreen_Opened[trueGreen_Opened < 254.5]  = 0
#    trueRed_Opened[trueRed_Opened < 254.5]      = 0
#    trueBlue_Opened[trueBlue_Opened < 254.5]    = 0
#    trueWhite_Opened[trueWhite_Opened < 254.5]  = 0

	 # For Green and Red ch: get the position of biggest shape  (Only one shape per ch) - and draw
    trueGreenDistance = cv2.distanceTransform(trueGreen_Opened,cv2.DIST_L2,0)
    trueRedDistance   = cv2.distanceTransform(trueRed_Opened,cv2.DIST_L2,0)
    
    gcentre = np.unravel_index(trueGreenDistance.argmax(),trueGreenDistance.shape)
    gradius = np.max(trueGreenDistance)
    rcentre = np.unravel_index(trueRedDistance.argmax(),trueRedDistance.shape)
    rradius = np.max(trueRedDistance)
    
    # Check if detected stuff is of right size
    if int(rradius) > RobArea and int(gradius) > RobArea:
        GreenCheck, RedCheck = 1,1
        cv2.circle(Positions,(gcentre[1],gcentre[0]),gradius, (0,255,0), 2)
        cv2.circle(Positions,(rcentre[1],rcentre[0]),rradius, (0,0,255), 2)
        cv2.line(Positions,(gcentre[1],gcentre[0]), (rcentre[1],rcentre[0]),(0,255,0), 2 )
        
    elif int(rradius) < RobArea and int(gradius) > RobArea:
        GreenCheck, RedCheck = 1,0
        cv2.circle(Positions,(gcentre[1],gcentre[0]),gradius, (0,255,0), 2)
        
    elif int(rradius) > RobArea and int(gradius) < RobArea:
        GreenCheck, RedCheck = 0,1
        cv2.circle(Positions,(rcentre[1],rcentre[0]),rradius, (0,255,0), 2)
        
    else:
        GreenCheck, RedCheck = 0,0

    # For Blue channel: find contours and centres - and draw
    cnts = cv2.findContours(trueBlue_Opened.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
         nCnts = np.shape(cnts)[0]
         WhiteList = np.zeros((len(cnts),2)) # Initialise array to list coords
         BlueCheck = 1
         for i in np.arange(0, len(cnts)):
             c = cnts[i]
             M = cv2.moments(c)
             area = cv2.contourArea(c)
             print(area)
             if area > TgtArea:
                 center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                 WhiteList[i,0], WhiteList[i,1] = center[1], center[0] # Add to list
                 cv2.circle(Positions,center,int(math.sqrt(int(area))), (255,0,0), 2)
         # If we have selected a target draw it on the map
         if TgtCheck == 1 and GreenCheck == 1:
             cv2.circle(Positions, (TgtCentre[1], TgtCentre[0]), 4,(100,100,255), 2)
             cv2.circle(Positions, (gcentre[1],gcentre[0]), TgtDistThres, (0, 150, 0), 1)
             cv2.line(Positions, (gcentre[1],gcentre[0]), (TgtCentre[1], TgtCentre[0]), (255, 255, 255), 1)
             cv2.putText(Positions, str(round(TgtAngle)), (20, 45),  cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2 )
             cv2. putText(Positions,'Tgt Dist' + str(math.sqrt(TgtDist)), (20, 65),  cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)
             # Display if we are aligned to target
             if AlignCheck == 1 :
                 cv2. putText(Positions, 'Aligned', (20, 85),  cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2 )
             else:
                 cv2. putText(Positions, 'Not Aligned', (20, 85),  cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2 )
         
    else: 
        # Write Check as 0s so that we can tell robot to random walk
        BlueCheck, TgtCheck, WhiteList = 0, 0, 0
        
             
         


    # For White channel: find contours and centres - and draw
    trueWhite_Opened = trueWhite_Opened.astype(np.uint8) #Need to convert to uint8
    cnts = cv2.findContours(trueWhite_Opened.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    if len(cnts) > 0:
         nCnts = np.shape(cnts)[0]
         WhiteCheck = 1 # We've found white targets
#         WhiteList = np.zeros((len(cnts),2)) # Initialise array to list coords
         
         for i in np.arange(0, len(cnts)): # Get coords of shapes + draw them
             c = cnts[i]
             M = cv2.moments(c)
             area = cv2.contourArea(c)
             if area > TgtArea and area < 4000:
                 center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                 # WhiteList[i,0], WhiteList[i,1] = center[1], center[0] # Add to list
                 cv2.circle(Positions,center,int(math.sqrt(int(area))), (255,255,255), 2)
             
#         # If we have selected a target draw it on the map
#         if TgtCheck == 1 and GreenCheck == 1:
#             cv2.circle(Positions, (TgtCentre[1], TgtCentre[0]), 4,(100,100,255), 2)
#             cv2.circle(Positions, (gcentre[1],gcentre[0]), TgtDistThres, (0, 150, 0), 1)
#             cv2.line(Positions, (gcentre[1],gcentre[0]), (TgtCentre[1], TgtCentre[0]), (255, 255, 255), 1)
#             cv2.putText(Positions, str(round(TgtAngle)), (gcentre[1]+3,gcentre[0]+3),  cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2 )
#             
#             # Display if we are aligned to target
#             if AlignCheck == 1 :
#                 cv2. putText(Positions, 'Aligned', (20, 20),  cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2 )
#             else:
#                 cv2. putText(Positions, 'Not Aligned', (20, 20),  cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2 )
#         
    else: 
#        # Write Check as 0s so that we can tell robot to random walk
        WhiteCheck = 0
        #, TgtCheck, WhiteList = 0, 0, 0
#        
    # Display what command we are giving the robot
    if command == 0:
        Cmdstr = 'Go - Kill'
        CmdCol = [100,100, 255]
    elif command == 1:
        Cmdstr = 'STOP'
        CmdCol = [0,0, 255]
    elif command == 2:
        Cmdstr = 'Go - Safe'
        CmdCol = [100,255, 100]
    elif command == 3:
        Cmdstr = 'Turn Right'
        CmdCol = [255,255, 255]
    elif command == 4:
        Cmdstr = 'Turn Left'
        CmdCol = [255,255, 255]
    elif command == 5:
        Cmdstr = 'Random Walk'
        CmdCol = [255,255, 255]
    elif command == 6:
        Cmdstr = 'Go back'
        CmdCol = [255,255, 255]
    else:
        Cmdstr = 'Confused???'
        CmdCol = [255,255, 255]
        
    cv2. putText(Positions,  Cmdstr, (20, 20),  cv2.FONT_HERSHEY_PLAIN, 1, CmdCol, 2)

    ### DISPLAY RESULTS ###
    frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
    cv2.imshow('frame',frame)
    cv2.imshow('Position',Positions )
#    cv2.imshow('trueGreen',trueGreen_Opened)
#    cv2.imshow('trueRed',trueRed_Opened)
#    cv2.imshow('trueBlue',trueBlue_Opened)
#    cv2.imshow('trueWhite',trueWhite_Opened )

    ### Exit if 'q' is pressed  ###
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()

    # In the end we can return relevant variables
    return GreenCheck, RedCheck, gcentre, rcentre, WhiteList, Positions, WhiteCheck, BlueCheck




###################
#### MAIN LOOP ####
###################

while(True):
	# Capture frame-by-frame
    ret, frame = cap.read()
    
    # Call Vision function
    GreenCheck, RedCheck, gcentre, rcentre, WhiteList, Positions, WhiteCheck, BlueCheck = Vision(frame, TgtCentre, TgtCheck, TgtAngle, AlignCheck, TgtDistThres, command, TgtDist, TgtArea )
    
    # Control animal behaviour
    if GreenCheck == 0 or RedCheck == 0:
        # We don't see the robot
        command = 5 # Ramdom walk if we dont have robot
        print('Random Walk')
    else: # We got the robot in sight
        if  BlueCheck == 1:    #WhiteCheck == 1:
            # Wer have robot and white shapes: select target
            WhiteDists, TgtCentre, TgtCheck, TgtAngle, TgtDist = TgtIdentif(gcentre, rcentre, WhiteList, Positions)
            if TgtCheck == 1:
                # We have a target selected, navigate to it
                AlignCheck, command = RobAligntoTgt(gcentre,TgtCentre, TgtAngle, TgtDist )
            else:
                command = 5 # Random walk if we haven't decided the target yet
                print('Random Walk')
        else:
            command = 5 # Ramdom walk if we dont have targets
            print('Random Walk')

#    We can send the command to the robot (if arduino is ready to listen )
    connected=False
#    while not connected:
#        serin = ser.read()
#        connected = True
    # Wait for the robot to respond to avoid overloading the buffer
    responded=False
    reference = bytes("8", 'utf8') # Reference to check if robot responded
    ser.write(bytes([7])) 
    counter = 0 # Count how many time it loops before getting a reply
    while not responded:
        counter = counter + 1
        if counter==5: # Need to re-run all other functions
            responded=True

        serin = ser.read() # Read serial
        ser.write(bytes([command])) # Send command to the robot
        # print(counter)
        if(serin==reference):
            # We can exit the while loop!
            ser.write(bytes([7])) 
            responded = True
            
    print(counter)


# Fin