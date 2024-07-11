import Neck
import Trunk
import Legs
import UpperArm
import LowerArm
import Wrist
import Load
import Scoring
import Person
import cv2


neck1 = Neck.Neck(10,True, False)
trunk1 = Trunk.Trunk(0,False,True)
legs1 = Legs.Legs(True,40)
upperArm1 = UpperArm.UpperArm(10, True, False, True, True)
lowerArm1 = LowerArm.LowerArm(70)
wrist1 = Wrist.Wrist(0,True)
load1 = Load.Load(7,False)
REBA_SCORE = 0


