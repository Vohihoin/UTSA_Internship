import cv2
import mediapipe as mp
import math
import Main
import Neck
import Trunk
import Legs
import UpperArm
import LowerArm
import Wrist
import Load
import Scoring
import Person
import VectorCalcs
import numpy as np

# Capture source is webcam number or video file path. 0 is inbuilt webcam and we start counting up from there
capture_source = 0

webcam = cv2.VideoCapture(capture_source)
mpPose = mp.solutions.pose
mpDraw = mp.solutions.drawing_utils
mpHands = mp.solutions.hands

pose = mpPose.Pose(static_image_mode=True,
    model_complexity=2,
    enable_segmentation=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)

hands = mpHands.Hands(False, 2)


neck1 = Neck.Neck(10,False, False)
trunk1 = Trunk.Trunk(0,False,False)
legs1 = Legs.Legs(True,10)
upperArm1 = UpperArm.UpperArm(10, False, False, False, False)
lowerArm1 = LowerArm.LowerArm(70)
wrist1 = Wrist.Wrist(0,False)
load1 = Load.Load(7,False)
Vahe = Person.Person(neck1,trunk1,legs1,load1,upperArm1,lowerArm1,wrist1,0,0)

# Smoothening utility variables
smootheningCounter = 0
firstRun = True
REBAScoreSum = 0


while True:

    ret, frame = webcam.read()
    dict_of_landmarks_raw = {}
    dict_of_landmarks = {}
    listOfHandDicts = []
    

    h, w, c = frame.shape

    #Processing
    frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frameRGB)
    handResults = hands.process(frameRGB)
    
    # Creating a list of dictionaries of landmarks
    
    if handResults.multi_hand_landmarks != None:

        for hand in handResults.multi_hand_landmarks:

            # drawing landmarks
            mpDraw.draw_landmarks(frame, hand, mpHands.HAND_CONNECTIONS)

            
            """
            We have two dictionaries,
            one for the hands
            the other for the dict of landmarks
            """

            workingLandmarkDict = {}
            for id, lm in enumerate(hand.landmark):

                workingLandmarkDict[id] = lm

            listOfHandDicts.append(workingLandmarkDict)

    

    #Creating a dict of pose landmarks for the person in frame
    
    if results.pose_landmarks != None:

        # Drawing landmarks
        mpDraw.draw_landmarks(frame, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
        dictOfHands = {}

        for id, lm in enumerate(results.pose_landmarks.landmark):
            dict_of_landmarks_raw[id] = lm
        
        
        # Converting the landmarks to actual image co-ordinates
        
        for index, key in enumerate(dict_of_landmarks_raw):

            lm = dict_of_landmarks_raw[key]
            dict_of_landmarks[key] = (int(lm.x * w), int(lm.y * h))

        pointsNecessary = True #boolean describing whether the necessary points for each section of measurement are available

        # Neck Code

        """
        Neck Angle

        The algorithm essentially uses the z positions of shoulders to determine if the person has turned
        Then, if they are turned, they use the ear and shoulder co-ordinates to get the angle of the neck to the vertical
        By first calculating slope, getting the angle and then shifting our reference to the vertical
        7 is left ear, 11 is left shoulder, 12 is right shoulder

        """

        pointsNecessary = 7 in dict_of_landmarks_raw and 8 in dict_of_landmarks_raw and 11 in dict_of_landmarks_raw and 12 in dict_of_landmarks_raw

        if pointsNecessary:

            # Utility points and vectors
            point7 = (dict_of_landmarks_raw[7].x, dict_of_landmarks_raw[7].y, dict_of_landmarks_raw[7].z) 
            point8 = (dict_of_landmarks_raw[8].x, dict_of_landmarks_raw[8].y, dict_of_landmarks_raw[8].z) 
            point11 = (dict_of_landmarks_raw[11].x, dict_of_landmarks_raw[11].y, dict_of_landmarks_raw[11].z) 
            point12 = (dict_of_landmarks_raw[12].x, dict_of_landmarks_raw[12].y, dict_of_landmarks_raw[12].z) 
            verticalUpVector = (0,-1,0)
            
            shoulderzdistance = dict_of_landmarks_raw[11].z - dict_of_landmarks_raw[12].z
            turned = abs(shoulderzdistance) > 0.5 #boolean to check if person is turned

            shoulder_midpoint = VectorCalcs.midPoint(point11, point12)
            eye_midpoint = VectorCalcs.midPoint(point7, point8)
            neckVector = VectorCalcs.getVector(shoulder_midpoint, eye_midpoint)

            neck_angle= VectorCalcs.getAngleBetweenVectors(neckVector, verticalUpVector)
            neck1.setAngle(neck_angle)
            #cv2.putText(frame, str(int(neck_angle)),  (400, 100), cv2.FONT_ITALIC, 0.6, (0,0,255))







        """

        Neck Twisted & Side Bending
        We're going to find the angle between the lines between the ears and those between the shoulders
        To find the angle between them to see if they're approximately parallel or not.

        Also, the angle measuring only works well if we can see the 23rd and 24th landmarks (hips). So, we'll check to see if they appear
        before checking for turning

        And if we are side-facing, comparing the eye landmarks to the shoulder landmarks works better than ear to shoulder.
        So, we'll use the turned boolean to decide which one to use
        
        """

        pointsNecessary = 7 in dict_of_landmarks_raw and 8 in dict_of_landmarks_raw and 11 in dict_of_landmarks_raw and 12 in dict_of_landmarks_raw
        
        if pointsNecessary:

            if (not turned):

                earL = (dict_of_landmarks_raw[7].x, dict_of_landmarks_raw[7].y, dict_of_landmarks_raw[7].z)
                earR = (dict_of_landmarks_raw[8].x, dict_of_landmarks_raw[8].y, dict_of_landmarks_raw[8].z)
                earVector = VectorCalcs.getVector(earL, earR)

                shoulderL = (dict_of_landmarks_raw[11].x, dict_of_landmarks_raw[11].y, dict_of_landmarks_raw[11].z)
                shoulderR = (dict_of_landmarks_raw[12].x, dict_of_landmarks_raw[12].y, dict_of_landmarks_raw[12].z)  
                shoulderVector = VectorCalcs.getVector(shoulderL, shoulderR) 


                neckTwistedAngle = VectorCalcs.getAngleBetweenVectors(shoulderVector, earVector)
                twistedActivationValue = 20

            else:

                mouthL = (dict_of_landmarks_raw[9].x, dict_of_landmarks_raw[9].y, dict_of_landmarks_raw[9].z)
                mouthR = (dict_of_landmarks_raw[10].x, dict_of_landmarks_raw[10].y, dict_of_landmarks_raw[10].z)
                mouthVector = VectorCalcs.getVector(mouthL, mouthR)

                shoulderL = (dict_of_landmarks_raw[11].x, dict_of_landmarks_raw[11].y, dict_of_landmarks_raw[11].z)
                shoulderR = (dict_of_landmarks_raw[12].x, dict_of_landmarks_raw[12].y, dict_of_landmarks_raw[12].z)  
                shoulderVector = VectorCalcs.getVector(shoulderL, shoulderR) 


                neckTwistedAngle = VectorCalcs.getAngleBetweenVectors(shoulderVector, mouthVector)
                twistedActivationValue = 7

            # cv2.putText(frame, "Neck Twisted Angle:  " + str(int(neckTwistedAngle)), (200,75), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0), 1)
        
            isTwisted = False
            isSideBending = False

       

            isTwisted = (neckTwistedAngle > twistedActivationValue) and (dict_of_landmarks_raw[23].y <= 1 and dict_of_landmarks_raw[24].y <= 1) # Making sure bottom joints are in frame
            isSideBending = isTwisted

            neck1.setTwisted(isTwisted)
        neck1.setSideBending(isSideBending)




        #Trunk
        """

        Trunk Angle

        To calculate this, we're going to use a vector space based on the i vector being the unit vector in the direction
        of media pose points 23 and 24



        """
  
        pointsNecessary = (11 in dict_of_landmarks_raw) and (12 in dict_of_landmarks_raw) and (23 in dict_of_landmarks_raw) and (24 in dict_of_landmarks_raw)
        
        if pointsNecessary:

            #Utility Variables

            point23 = (dict_of_landmarks_raw[23].x, dict_of_landmarks_raw[23].y, dict_of_landmarks_raw[23].z) 
            point24 = (dict_of_landmarks_raw[24].x, dict_of_landmarks_raw[24].y, dict_of_landmarks_raw[24].z) 
            hip_midpoint = VectorCalcs.midPoint(point23, point24)

            point11 = (dict_of_landmarks_raw[11].x, dict_of_landmarks_raw[11].y, dict_of_landmarks_raw[11].z) 
            point12 = (dict_of_landmarks_raw[12].x, dict_of_landmarks_raw[12].y, dict_of_landmarks_raw[12].z) 
            shoulder_midpoint = VectorCalcs.midPoint(point11, point12)

            
            # hts - hip to shoulder
            # h - hip
            # s - shoulder
            # hpen - hip perpendicular (vector that is perpendicular to the hip)

            htsVector = VectorCalcs.getVector(hip_midpoint, shoulder_midpoint)
            hVector = VectorCalcs.getVector(point24, point23)
            sVector = VectorCalcs.getVector(point12, point11)

        

            # angle calcs
            hPlanarVector = ( point12[0]-point11[0], 0, (point11[2]-point12[2]) )# y is 0 because vector is planar
            hpenVector = (hPlanarVector[2], 0, -hPlanarVector[0]) # we flip z and x and make the new z negative since we're rotating about the y-axis

            verticalUpVector = (0,-1,0)
        

            trunk_angle = VectorCalcs.getAngleBetweenVectors(verticalUpVector, htsVector)
            trunk1.setAngle(trunk_angle)
            #cv2.putText(frame, str(trunk_angle), (200,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)

    
            """
    
            Trunk Twisted and Side-bending. Similar approach to side bending for neck.
            Meause angle between shoulder vector and hip vector
    
            """

            # still need to tune the threshold angle
            thresholdAngle = 25

            trunk_twisted_angle = VectorCalcs.getAngleBetweenVectors(hVector, sVector)
            trunkTwisted = trunk_twisted_angle > thresholdAngle
            trunkSideBending = trunkTwisted

            trunk1.setTwisted(trunkTwisted)
            trunk1.setSideBending(trunkSideBending)

        # Legs
        
        """
        Legs Down

        To get this, we compare the y-position of landmarks 29 and 30

        """

        pointsNecessary = (29 in dict_of_landmarks_raw) and (30 in dict_of_landmarks_raw)

        if pointsNecessary:
            point29 = (dict_of_landmarks_raw[29].x, dict_of_landmarks_raw[29].y, dict_of_landmarks_raw[29].z) 
            point30 = (dict_of_landmarks_raw[30].x, dict_of_landmarks_raw[30].y, dict_of_landmarks_raw[30].z) 
            heelDifferenceThreshold = 0.1

            heelDifference = abs(point29[1] - point30[1])
            legsDown = not (heelDifference > heelDifferenceThreshold) # still needs to be tuned

            #cv2.putText(frame, str(round(heelDifference, 3)), (100,100), cv2.FONT_ITALIC, 1, (0,0,255))

            legs1.setLegsDown(legsDown)

    
        """
        Legs Angle

        We're going to use the hPen vector from the trunk code before for this because that vector is basically
        the body's direction vector. So, it can be used for measuring the angles.

        Then depending on whether the left or right side is closer, we either use the left or right side to get the thigh vector

        """

        pointsNecessary = 23 in dict_of_landmarks_raw and 24 in dict_of_landmarks_raw and 25 in dict_of_landmarks_raw and 26 in dict_of_landmarks_raw

        if pointsNecessary:

            point23 = (dict_of_landmarks_raw[23].x, dict_of_landmarks_raw[23].y, dict_of_landmarks_raw[23].z) 
            point24 = (dict_of_landmarks_raw[24].x, dict_of_landmarks_raw[24].y, dict_of_landmarks_raw[24].z) 
            point25 = (dict_of_landmarks_raw[25].x, dict_of_landmarks_raw[25].y, dict_of_landmarks_raw[25].z) 
            point26 = (dict_of_landmarks_raw[26].x, dict_of_landmarks_raw[26].y, dict_of_landmarks_raw[26].z)

            verticalUpVector = (0,-1,0)

            leftSideCloser= point25[2] < point26[2]  
    
            if leftSideCloser:

                thighVector = VectorCalcs.getVector(point25, point23)

            else:

                thighVector = VectorCalcs.getVector(point26, point24)

    
            leg_angle = (VectorCalcs.getAngleBetweenVectors(thighVector, verticalUpVector)) 
            
            legs1.setAngle(leg_angle)
            #cv2.putText(frame, str(int(leg_angle)), (500,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
            


        
        # Upper Arm
            
        """
        Upper Arm Angle

        Measure angle between our upper arm vector and vertical

        """
        pointsNecessary = 11 in dict_of_landmarks_raw and 12 in dict_of_landmarks_raw and 13 in dict_of_landmarks_raw and 14 in dict_of_landmarks_raw
        if pointsNecessary:

            point11 = (dict_of_landmarks_raw[11].x, dict_of_landmarks_raw[11].y, dict_of_landmarks_raw[11].z) 
            point12 = (dict_of_landmarks_raw[12].x, dict_of_landmarks_raw[12].y, dict_of_landmarks_raw[12].z) 
            point13 = (dict_of_landmarks_raw[13].x, dict_of_landmarks_raw[13].y, dict_of_landmarks_raw[13].z) 
            point14 = (dict_of_landmarks_raw[14].x, dict_of_landmarks_raw[14].y, dict_of_landmarks_raw[14].z) 

            verticalDownVector = (0, 1, 0)
            leftSideCloser= point11[2] < point12[2]

            if leftSideCloser:

                upperArmVector = VectorCalcs.getVector(point11,point13)
            else:
                upperArmVector = VectorCalcs.getVector(point12, point14)

            upperArmAngle = VectorCalcs.getAngleBetweenVectors(upperArmVector, verticalDownVector)
            upperArm1.setAngle(upperArmAngle)


            #cv2.putText(frame, str(int(upperArmAngle)), (100,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)

        #Lower Arm
        """

        Lower Arm Angle

        """

        pointsNecessary = 13 in dict_of_landmarks_raw and 14 in dict_of_landmarks_raw and 15 in dict_of_landmarks_raw and 16 in dict_of_landmarks_raw

        if pointsNecessary:

            point13 = (dict_of_landmarks_raw[13].x, dict_of_landmarks_raw[13].y, dict_of_landmarks_raw[13].z) 
            point14 = (dict_of_landmarks_raw[14].x, dict_of_landmarks_raw[14].y, dict_of_landmarks_raw[14].z) 
            point15 = (dict_of_landmarks_raw[15].x, dict_of_landmarks_raw[15].y, dict_of_landmarks_raw[15].z) 
            point16 = (dict_of_landmarks_raw[16].x, dict_of_landmarks_raw[16].y, dict_of_landmarks_raw[16].z) 
            verticalDownVector = (0, 1, 0)

            leftSideCloser= point15[2] < point16[2]

            if leftSideCloser:

                lowerArmVector = VectorCalcs.getVector(point13, point15)

            else:

                lowerArmVector = VectorCalcs.getVector(point14,point16)

            lowerArmAngle = VectorCalcs.getAngleBetweenVectors(lowerArmVector, verticalDownVector)
            lowerArm1.setAngle(lowerArmAngle)

        
        """
        
        Wrist Angle and Twisted
        Wrist Angle:- We create a wrist vector and measure it's angle from the vertical and use that to figure out the angle from
        the horizontal.

        To get our wrist angle, we want to use mediapipe's hand solutions because is should give us more accurate 
        data about the hand co-ordinates. So, our handVector will be based on points 0 and 12 of mediapipe's hand landmarks

        We also need to figure out which hand is left or right

        """



        pointsNecessary = (15 in dict_of_landmarks_raw) and (handResults.multi_hand_landmarks != None) and (len(handResults.multi_hand_landmarks) > 1) and (13 in dict_of_landmarks_raw) and (14 in dict_of_landmarks_raw) and (16 in dict_of_landmarks_raw)
        if pointsNecessary:


            point15 = (dict_of_landmarks_raw[15].x, dict_of_landmarks_raw[15].y, dict_of_landmarks_raw[15].z) # left palm marker
            point13 = (dict_of_landmarks_raw[13].x, dict_of_landmarks_raw[13].y, dict_of_landmarks_raw[13].z) 
            point14 = (dict_of_landmarks_raw[14].x, dict_of_landmarks_raw[14].y, dict_of_landmarks_raw[14].z) 
            point16 = (dict_of_landmarks_raw[16].x, dict_of_landmarks_raw[16].y, dict_of_landmarks_raw[16].z) 
            

            leftLowerArmVector = VectorCalcs.getVector(point15, point13)
            rightLowerArmVector = VectorCalcs.getVector(point16, point14)

            
            #We're going to figure out which hand is left and right by comparing the distance between the palm marker for left from
            # the pose estimation to the palm marker of left hand and right hand from the hand estimation and seeing which ones are closer

            handDistances = []
            handDistances = [VectorCalcs.getDistanceBetweenPoints(point15, (hand[0].x, hand[0].y, hand[0].z)) for hand in listOfHandDicts]
            leftHandID = handDistances.index(min(handDistances))
            
            
            if leftHandID == 0:
                rightHandID = 1

            elif leftHandID == 1:
                rightHandID = 0

            leftHandPoint0 = (listOfHandDicts[leftHandID][0].x, listOfHandDicts[leftHandID][0].y,listOfHandDicts[leftHandID][0].z)
            leftHandPoint9 = (listOfHandDicts[leftHandID][9].x, listOfHandDicts[leftHandID][9].y,listOfHandDicts[leftHandID][9].z)

            rightHandPoint0 = (listOfHandDicts[rightHandID][0].x, listOfHandDicts[rightHandID][0].y,listOfHandDicts[rightHandID][0].z)
            rightHandPoint9 = (listOfHandDicts[rightHandID][9].x, listOfHandDicts[rightHandID][9].y,listOfHandDicts[rightHandID][9].z)

            leftHandVector = VectorCalcs.getVector(leftHandPoint0, leftHandPoint9)
            rightHandVector = VectorCalcs.getVector(rightHandPoint0, rightHandPoint9)

            right_wrist_angle = (180 - VectorCalcs.getAngleBetweenVectors(rightLowerArmVector, rightHandVector))
            left_wrist_angle = (180 - VectorCalcs.getAngleBetweenVectors(leftLowerArmVector, leftHandVector))

            cv2.putText(frame, "Right Wrist Angle : " + str(int(right_wrist_angle)), (100,50), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)
            cv2.putText(frame, "Left Wrist Angle " +str(int(left_wrist_angle)), (100,100), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)


            # we use the worst angle among the wrists to set our angle

            if (abs(left_wrist_angle) > abs(right_wrist_angle)):
                wristAngle = left_wrist_angle

            else:
                wristAngle = right_wrist_angle

        
            wrist1.setAngle(wristAngle)

    # Update all the body part values
    Vahe = Person.Person(neck1,trunk1,legs1,load1,upperArm1,lowerArm1,wrist1,0,0)
    #cv2.putText(frame, "Right Wrist Angle : " + str(int(right_wrist_angle)), (100,50), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)
    #cv2.putText(frame, "Left Wrist Angle " +str(int(left_wrist_angle)), (100,100), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)
    #cv2.putText(frame, "ScoreA: " +str(Vahe.getScoreA()), (100,150), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)
    #cv2.putText(frame, "ScoreB: " +str(Vahe.getScoreB()), (100,200), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)
    #cv2.putText(frame, "ScoreC: " +str(Vahe.getScoreC()), (100,250), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)


    # Smoothening Utility to Prevent Fluctuation of REBA Score
    smootheningFrames = 10

    if smootheningCounter < smootheningFrames:
        REBAScoreSum += Vahe.getREBAScore()
        smootheningCounter += 1
    else:
        REBAScore = REBAScoreSum/smootheningFrames
        REBAScoreSum = 0
        smootheningCounter = 0


    if firstRun:
        REBAScore = Vahe.getREBAScore()
        firstRun = False


    cv2.putText(frame, "REBA Score: " +str(REBAScore), (100,300), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0,0,255), 1)
    cv2.imshow('WINDOW', frame)

    if cv2.waitKey(5) & 0XFF==ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()
exit()