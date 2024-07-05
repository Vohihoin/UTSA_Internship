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
import hellopose
import VectorCalcs

webcam = cv2.VideoCapture(0)
mpPose = mp.solutions.pose
mpDraw = mp.solutions.drawing_utils
pose = mpPose.Pose()

neck1 = Neck.Neck(10,True, False)
trunk1 = Trunk.Trunk(0,False,True)
legs1 = Legs.Legs(True,40)
upperArm1 = UpperArm.UpperArm(10, True, False, True, True)
lowerArm1 = LowerArm.LowerArm(70)
wrist1 = Wrist.Wrist(0,True)
load1 = Load.Load(7,False)
Vahe = Person.Person(neck1,trunk1,legs1,load1,upperArm1,lowerArm1,wrist1,0,0)


while True:

    ret, frame = webcam.read()
    dict_of_landmarks_raw = {}
    dict_of_landmarks = {}

    h, w, c = frame.shape

    #Processing
    frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frameRGB)
    
    if results.pose_landmarks != None:

        # Drawing landmarks
        mpDraw.draw_landmarks(frame, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
        
        for id, lm in enumerate(results.pose_landmarks.landmark):
            dict_of_landmarks_raw[id] = lm
        
        # Converting the landmarks to actual image co-ordinates
        
        for index, key in enumerate(dict_of_landmarks_raw):

            lm = dict_of_landmarks_raw[key]
            dict_of_landmarks[key] = (int(lm.x * w), int(lm.y * h))
        

        # Neck Code

        """
        Neck Angle

        The algorithm essentially uses the z positions of shoulders to determine if the person has turned
        Then, if they are turned, they use the ear and shoulder co-ordinates to get the angle of the neck to the vertical
        By first calculating slope, getting the angle and then shifting our reference to the vertical

        """
        shoulderzdistance = abs(dict_of_landmarks_raw[11].z - dict_of_landmarks_raw[12].z)
        turned = shoulderzdistance > 0.5 #boolean to check if person is turned

    
        # 7 is left ear, 11 is left shoulder
        if turned:
            rise = dict_of_landmarks[11][1] - dict_of_landmarks[7][1]
            run = dict_of_landmarks[11][0] - dict_of_landmarks[7][0]

            if run != 0:

                m = (rise)/(run) # slope, rise over run
                neck_angle = int(90 - math.degrees(math.atan(m)))

            else:
                neck_angle = 0

            
            if neck_angle > 90:
                neck_angle = -(180 - neck_angle)

            neck1.setAngle(neck_angle)
            cv2.putText(frame, str(neck_angle), (400,50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), 1)

        else:

            rise = dict_of_landmarks_raw[11].y - dict_of_landmarks_raw[7].y
            run = dict_of_landmarks_raw[11].z - dict_of_landmarks_raw[7].z

            if run != 0:

                m = (rise)/(run) # slope, rise over run
                neck_angle = int(90 - math.degrees(math.atan(m)))

            else:

                neck_angle = 0

            
            if neck_angle > 90:
                neck_angle = -(180 - neck_angle)

            neck1.setAngle(neck_angle)
            cv2.putText(frame, str(neck_angle), (400,50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), 1)


        """

        Neck Twisted
        We're going to find the angle between the lines between the ears and those between the shoulders
        To find the angle between them to see if they're approximately parallel or not.

        Also, the angle measuring only works well if we can see the 23rd and 24th landmarks (hips). So, we'll check to see if they appear
        before checking for turning

        And if we are side-facing, comparing the eye landmarks to the shoulder landmarks works better than ear to shoulder.
        So, we'll use the turned boolean to decide which one to use
        
        """

        if not turned:

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

       

        isTwisted = (neckTwistedAngle > twistedActivationValue) and (dict_of_landmarks_raw[23].y <= 1 and dict_of_landmarks_raw[24].y <= 1) # Making sure bottom joints are in frame
        if isTwisted:
                cv2.putText(frame, "Neck Twisted " + str(int(neckTwistedAngle)), (200,75), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0), 1)

        neck1.setTwisted(isTwisted)
        
    
    Vahe = Person.Person(neck1,trunk1,legs1,load1,upperArm1,lowerArm1,wrist1,0,0)
    cv2.putText(frame, str(Vahe.getREBAScore()), (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)
    

    cv2.imshow('WINDOW', frame)

    if cv2.waitKey(5) & 0XFF==ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()
exit()