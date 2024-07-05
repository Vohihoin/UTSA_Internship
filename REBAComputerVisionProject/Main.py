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
import cv2


neck1 = Neck.Neck(10,True, False)
trunk1 = Trunk.Trunk(0,False,True)
legs1 = Legs.Legs(True,40)
upperArm1 = UpperArm.UpperArm(10, True, False, True, True)
lowerArm1 = LowerArm.LowerArm(70)
wrist1 = Wrist.Wrist(0,True)
load1 = Load.Load(7,False)
REBA_SCORE = 0

while True:

    neck1.setAngle(hellopose.neck_angle)
    Vahe = Person.Person(neck1,trunk1,legs1,load1,upperArm1,lowerArm1,wrist1,0,0)
    REBA_SCORE = Vahe.getREBAScore()
