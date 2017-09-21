
#########################
### BEHAVIOUR CONTROL ###
#########################

# The robot needs to: 
# 	identify target 	   - choose closes baloon of correct colors
# 	plot trajectory to tgt - find relative angle, turn the robot to face tgt
# 	move towards tgt 	   - avoid obstacles on the way (maybe local code on bot)
# 	kill 				   - stop in front of tgt facing it, activate servo, confirm kill
# 	... repeat ... 

# Exceptions: 
# 	robot is stuck 	   - check if not moving
# 	robot is lost 	   - visual recognition is not working
# 	no target          - visual recog not working or can't make a decision 
# 	tgt not killed 	   - the killing is not sucesful, find alternative


#############################
#### BEHAVIOUR FUNCTIONS ####
#############################

# *** LIST OF FUNCTIONS:
# * Stop robot
# 	returns ok if robot is stopped

# * Random walk
# 	returns ok if robot is random waling
# 	Robot should be random walking
# 	Using proximity sensors to avoid obstacles
# 	If we can see one of the two markers, check if robot stuck?

# * Target identification
# 	returns ok if a tgt is identified 
# 	Uses robot location and tgt baloons location to find closes baloon
# 	If two baloons are at similar distance, privilege the one closest to alignment?

# * Align to Tgt 
# 	reuturns ok if robot is aligned to tgt
# 	Calculate relative angle of tgt to robot, needs to be below a set threshold
# 	If above threshold rotate accordingly

# * Navigation
# 	Within this function need to continously check for alignment and proximity
# 	If we are aligned and not too close: move forward
# 	Robot should be able to override commands when an obstacle appears, but not when this obstacle is our target

# * Proximity to target
# 	check if robot is close to target and aligned to it
# 	if it is: stop the robot and check again

# * Killing
# 	Activate servo to move arm that pops baloons
# 	Confirm kill: the balloon disappeared and a contour of different shape/size appeared where it was
# 	If kill confirmed, back of a while to stay clear of debries?
# 	If kill NOT confirmed, try again for a number of times, otherwise give up and go after a different tgt


# ** List of Ancillary functions: 
# * Connection lost
# 	The robot's code should stop in this case
# 	stop everything until we can re-establish the Connection

# * Robot confuse
# 	Set all behaviour flags as zero and restart main loop

# * Program terminated
# 	Stop robot
# 	Close camera acquisition 

# * Task Completed
# 	Victory dance
# 	Program Terminated


#### IMPORT ####
import numpy as np
import cv2
from vectors import Point, Vector
from fn.uniform import reduce
import math


#### SET UP ####
# Creat camera 
cap = cv2.VideoCapture(0)

# Thresholds for channel filtering 
smallTh     = 220 	
blueSmallTh = 220 	# Different small Th for blue channel (different saturation?)
bigTh       = 255

# Number of baloons and kills (might be useful for tracking and counting number of kills)
WhiteBaloons     = 5
BlueBaloons      = 5
WhiteKillCounter = 0
BlueKillCounter  = 0
TgtArea 		 = 800 # Min area of a baloon 
RobArea      = 5 # Min area of robots lables

# Flags
WhiteCheck, TgtCheck = 0,0 # Equal 0 if we don't have a tgt
TgtCentre = [0,0]
TgtAngle = 0; 
## Functions

def RobStop():
	a = 1
	# Inputs: 

	# Stop Robot

	# Output: ok from robot

def RandomWalk():
	a = 1 
	# Inputs: 

	# Random walk
	# Check if we have found the robot 
	# Check if robot stuck: if so go back 

	# Output: ok from robot

def TgtIdentif(gcentre, rcentre, WhiteList, Positions):

    ## Find closest white target
    robVec= [gcentre[1]-rcentre[1],gcentre[0]-rcentre[0]]
    
    WhiteDists = np.zeros((len(WhiteList),1))
    for i in np.arange(0, len(WhiteList)):
        whiteVec = [WhiteList[i,1]-gcentre[1],WhiteList[i,0]-gcentre[0]]
        WhiteDists[i] =  whiteVec[0]**2 + whiteVec[1]**2
        
        WhiteDists = WhiteDists.astype(np.uint16)
        MinDist = np.argmin(WhiteDists)
        
        TgtCentre = WhiteList[MinDist,:]
        TgtCentre = TgtCentre.astype(np.uint16)
        
    
    ## Find angle to target
    tgtVec = [TgtCentre[1]-gcentre[1], TgtCentre[0]-gcentre[0]]
    
    TgtAngle = angle_between(robVec, tgtVec)  # Angle in radian
    TgtAngle = TgtAngle * 180 / math.pi       # Angle in degrees
    TgtAngle = ( TgtAngle +180 ) % 360 - 180  # Remap to values between -179 and +180
    
    TgtCheck = 1

    return WhiteDists, TgtCentre, TgtCheck, TgtAngle

def RobAligntoTgt(): 
	a = 1
	# Inputs: Rob location, Rob orientation, tgt location

	# Compute relative angle
	# Check if below threshold
	# Rotate robot correctly until angle below threshold	

	# Output: Ok for align correct

def Navigation():
	a = 1
	# Inputs: Rob location, Rob orientation, tgt location

	# Check at regular intervals:
		# If tgt and robot arre still identified
			# else need to trigger vision function 
		# If alignemnt to tgt is correct: RobAligntoTgt
		# if proximity to tgt is reached: 
			# if reached trigger Kill
		# if proximity to obstacle is reached?

	# If all good keep moving forward (Rob should override this when an obstacle is encountered)


def Kill():
	a = 1
	# Input: robot location, tgt location, (proximity readout from Rob?)

	# RobStop

	# Activate killing arm servo
	# Check if kill succesful - else repeat n time
	# Move to next tgt

	# Output: kill success check


def ProxtoObst():
	a = 1
	# not sure if useful, find close obstacles

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    


def Vision(frame, smallTh, blueSmallTh, bigTh, WhiteBaloons, BlueBaloons, WhiteKillCounter, BlueKillCounter, TgtCentre, TgtCheck, TgtAngle): 

    # Inputs: camera frames, thresholds
    # Outputs: robot, tgts, obsts locations + checks

    ## Setting up ##
    # Create empty array to display results
    Positions = np.zeros((np.shape(frame)))
    
    ## Filtering Channels ##
    # Get Individual channels and threshold them 
    blue  = frame[:,:,0]
    green = frame[:,:,1]
    red   = frame[:,:,2]
    
    retB, blueTh   = cv2.threshold(blue, smallTh, blueSmallTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retB, blueThff = cv2.threshold(blue, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C) # If we want to subtract blue to other ch, we need differently filtered version
    retG, greenTh  = cv2.threshold(green, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    retR, redTh    = cv2.threshold(red, smallTh, bigTh, cv2.ADAPTIVE_THRESH_MEAN_C)
    
    # Do a first morpholical filtering to eliminate noise across channels 
    Element1  = np.ones((15,15))
    Element4  = np.ones((25,25))
    Gelement2 = np.ones((7,7))
    Relement1 = np.ones((15,15))
    Relement2 = np.ones((7,7))
    Relement3 = np.ones((45,45))

    greenDilated = cv2.morphologyEx(greenTh,cv2.MORPH_DILATE,Element1)
    redDilated   = cv2.morphologyEx(redTh,cv2.MORPH_DILATE,Element1)
    blueDilated  = cv2.morphologyEx(blueThff,cv2.MORPH_DILATE,Element1)
    
    # Subtract filtered channels to get true colors
    trueGreen  = greenTh - blueDilated - redDilated
    trueBlue   = blueThff - greenDilated - redDilated
    trueRed    = redTh - blueDilated     
    trueWhite  = np.round(greenTh/3 + redTh/3 + blueThff/3)

    # Threshold channels correclty to eliminate odd values
    trueGreen[trueGreen < 0]    = 0
    trueBlue[trueBlue < 0]      = 0
    trueRed[trueRed < 0]        = 0
    trueWhite[trueWhite < 250]  = 0
    trueWhite[trueWhite >= 250] = 255

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
    trueGreen_Opened[trueGreen_Opened < 254.5]  = 0
    trueRed_Opened[trueRed_Opened < 254.5]      = 0
    trueBlue_Opened[trueBlue_Opened < 254.5]    = 0
    trueWhite_Opened[trueWhite_Opened < 254.5]  = 0

	# For Green and Red ch: get the position of biggest shape  (Only one shape per ch) - and draw
    trueGreenDistance = cv2.distanceTransform(trueGreen_Opened,cv2.DIST_L2,0)
    trueRedDistance   = cv2.distanceTransform(trueRed_Opened,cv2.DIST_L2,0)
    
    gcentre = np.unravel_index(trueGreenDistance.argmax(),trueGreenDistance.shape)
    gradius = np.max(trueGreenDistance)
    rcentre = np.unravel_index(trueRedDistance.argmax(),trueRedDistance.shape)
    rradius = np.max(trueRedDistance)

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
         # cnt = cnts[0]
         # cv2.drawContours(Positions,cnts, -1, (255,0,0), 3 )
         for i in np.arange(0, len(cnts)):
             c = cnts[i]
             area = cv2.contourArea(c)
             if area > TgtArea: # Filetirng out small contours 
                 M = cv2.moments(c)
                 center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                 cv2.circle(Positions,center,25, (255,0,0), 2)

                 # !! If kills = 0, check if number of blue shapes matches known number of tgts
                 # !! Need to save centres coords to some data structure to pass to functions

         BlueCheck = 1
    else: 
    	# No blue shapes identified: either recognition not working or all targets disappeared
    	# If we can't find any blue baloon and there should be some, finding them should be priority - stop robot
    	if BlueKillCounter < BlueBaloons: #We haven't yet killed all blue baloons
    		BlueCheck = 0
    	else:
    		a = 1
    		# We killed all of them 


    # For White channel: find contours and centres - and draw
    trueWhite_Opened = trueWhite_Opened.astype(np.uint8) #Need to convert to uint8
    cnts = cv2.findContours(trueWhite_Opened.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
         nCnts = np.shape(cnts)[0]

         WhiteCheck = 1 # We've found white targets
         WhiteList = np.zeros((len(cnts),2)) # Initialise array to list coords
         for i in np.arange(0, len(cnts)):
             c = cnts[i]
             # Might need to change size filtering here, similiar to blue channel 
             M = cv2.moments(c)
             center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
             WhiteList[i,0], WhiteList[i,1] = center[1], center[0] # Add to list
             cv2.circle(Positions,center,25, (255,255,255), 2)
             
         if TgtCheck == 1:
             cv2.circle(Positions, (TgtCentre[1], TgtCentre[0]), 4,(100,100,255), 2)
             cv2.line(Positions, (gcentre[1],gcentre[0]), (TgtCentre[1], TgtCentre[0]), (255, 255, 255), 1)
             cv2.putText(Positions, str(round(TgtAngle)), (gcentre[1]+3,gcentre[0]+3),  cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2 )
         
    else: 
    	# No white shapes identified: either recognition not working or all targets disappeared
    	# Use WhiteBaloons and WhiteKillCounter to know if there should be any
		# If we can't find any white baloon and there should be some, finding them should be priority - stop robot
        WhiteCheck, TgtCheck, WhiteList = 0, 0, 0

    ### DISPLAY RESULTS ###
    cv2.imshow('frame',frame)
    cv2.imshow('Position',Positions )

    #cv2.imshow('W',trueWhite_Opened)
    #cv2.imshow('GG',greenTh)
    #cv2.imshow('R',trueRed)
    #cv2.imshow('b', trueBlue)
    #cv2.imshow('tb', blueTh)
    #cv2.imshow('tbo',trueBlue_Opened)
    #cv2.imshow('G',trueGreen)
    #cv2.imshow('Gopened',trueGreen_Opened)
    #cv2.imshow('Ropened',trueRed_Opened)
    #cv2.imshow('GdT',trueGreenDistance)
    
    ### Exit if 'q' is pressed  ###
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ### When everything done, release the capture ###
        cap.release()
        cv2.destroyAllWindows()

    # If everything went well we can return relevant variables
    return GreenCheck, RedCheck, gcentre, rcentre, WhiteList, Positions, WhiteCheck






###################
#### MAIN LOOP ####
###################

while(True):
	# Capture frame-by-frame
    ret, frame = cap.read()

    GreenCheck, RedCheck, gcentre, rcentre, WhiteList, Positions, WhiteCheck = Vision(frame, smallTh, blueSmallTh, bigTh, WhiteBaloons, BlueBaloons, WhiteKillCounter, BlueKillCounter, TgtCentre, TgtCheck, TgtAngle)

    if GreenCheck == 0 or RedCheck == 0:
     	# We can't find one or both of the  markes on the robot
     	# We should stop the robot or random walk until we can find it again
         
         
#     robStop()
#     RandomWalk()  # For some time or until the robot is found again  
         a = 1
    else:
        if WhiteCheck == 1:
            WhiteDists, TgtCentre, TgtCheck, TgtAngle = TgtIdentif(gcentre, rcentre, WhiteList, Positions)
        
        
#   	 	# We know where the robot is: we can keep going with controlling the robot's behaviour
#   	 	if BlueCheck == 1: # or WhiteCheck == 1, depending on which one is tgt
#   	 		# We know where the robot is and we know where at least one tgt baloon is 
#   	 		TgtIdentif() # Find target, returns TgtCheck = 1 if found 
#
#   	 		if TgtCheck == 1: # We have a tgt
#   	 			# Align to target
#   	 			RobAligntoTgt()	# Returns AlignCheck when alignment correct
#
#   	 			if AlignCheck == 1: # We are aligned to tgt
#   	 				Navigation() # Will move the robot until it's close to tgt and facing it 
#   	 							 # When in position it triggers the Kill function 
#
#   	 	else: # we don't have an identified blue baloon
#   	 		RobStop()


# Fin