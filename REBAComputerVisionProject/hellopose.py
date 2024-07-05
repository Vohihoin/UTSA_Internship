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

            neck1.setAngle(neck_angle)
            Vahe = Person.Person(neck1,trunk1,legs1,load1,upperArm1,lowerArm1,wrist1,0,0)
            cv2.putText(frame, str(Vahe.getREBAScore()), (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)
            cv2.putText(frame, str(neck_angle), (400,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)


        #cv2.circle(frame, dict_of_landmarks[12], 10, (255,0,150), -1)
    
    

    cv2.imshow('WINDOW', frame)

    if cv2.waitKey(5) & 0XFF==ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()