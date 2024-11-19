import cv2
import mediapipe 
import numpy

#Class resources
source = 0
webcam = cv2.VideoCapture(source)
mpPose = mediapipe.solutions.pose
pose = mpPose.Pose(
    static_image_mode=True,
    model_complexity=2,
    enable_segmentation=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mpDraw = mediapipe.solutions.drawing_utils


"""
Method that does one run (cycle) of the frame processing process
Captures a frame from the specified source and produces the results for it
Returns -1 if the method isn't able to capture a frame or 
Returns -2 it doesn't have enough data to work with (not enough is in frame)
"""
def run(source):

    landmarks = []
    visibleLandmarks = []
    webcam = cv2.VideoCapture(source)
    ret, frame = webcam.read()

    if (not ret):
        return -1
    
    frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    bodyResults = pose.process(frameRGB)
    mpDraw.draw_landmarks(frame, bodyResults.pose_landmarks, mpPose.POSE_CONNECTIONS)
    cv2.imshow("WINDOW",frame)

    for ldmk in enumerate(bodyResults.pose_landmarks.landmark):
        landmarks.append(ldmk) 
        # each landmark object is a tuple of form:
        # (0, x: 0.604244828
        # y: 0.413355529
        # z: -0.458691299
        # visibility: 0.999419332
        # )
    
    
    visibleLandmarks = [ldmk for ldmk in landmarks if ldmk[1].visibility > 0.9]
   
    
    containsEnough = ( containsLandmark(visibleLandmarks, 0) 
                      and containsLandmark(visibleLandmarks, 1) 
                      and containsLandmark(visibleLandmarks, 2)
                      and containsLandmark(visibleLandmarks, 3)
                      and containsLandmark(visibleLandmarks, 4)
                      and containsLandmark(visibleLandmarks, 5)
                      and containsLandmark(visibleLandmarks, 6)
                      and containsLandmark(visibleLandmarks, 7)
                      and containsLandmark(visibleLandmarks, 8)
                      and containsLandmark(visibleLandmarks, 9)
                      and containsLandmark(visibleLandmarks, 10)
                      and containsLandmark(visibleLandmarks, 11)
                      and containsLandmark(visibleLandmarks, 12))

    if (not containsEnough):
        return -2
    

    


def containsLandmark(listLandmarks, ldmkNum):

    # each landmark object is a tuple of form:
    # (0, x: 0.604244828
    # y: 0.413355529
    # z: -0.458691299
    # visibility: 0.999419332
    # )

    for ldmk in listLandmarks:
        if ldmk[0] == ldmkNum:
            return True
        
    return False



def testAccuracy(listLandmarks):

    # each landmark object is a tuple of form:
    # (0, x: 0.604244828
    # y: 0.413355529
    # z: -0.458691299
    # visibility: 0.999419332
    # )

    actualShoulderLength = 5
    measuredShoulderValue = distanceBtwLandmarks(getLandmark(listLandmarks, 12), getLandmark(listLandmarks, 11))

    actualEarLength = 5
    measuredEarValue = distanceBtwLandmarks(getLandmark(listLandmarks, 8), getLandmark(listLandmarks, 7))

    actualMouthLength = 5
    measuredMouthValue = distanceBtwLandmarks(getLandmark(listLandmarks, 10), getLandmark(listLandmarks, 9))

    actualEyeLength = 5
    measuredEyeValue = distanceBtwLandmarks(getLandmark(listLandmarks, 5), getLandmark(listLandmarks, 2))

    print(""+(actualEarLength/measuredEarValue)+", "+(actualMouthLength/measuredMouthValue)+", "+(actualEyeLength/measuredEyeValue))


def getLandmark(listLandmarks, ldmkNum):

    # each landmark object is a tuple of form:
    # (0, x: 0.604244828
    # y: 0.413355529
    # z: -0.458691299
    # visibility: 0.999419332
    # )

    for ldmk in listLandmarks:
        if ldmk[0] == ldmkNum:
            return ldmk
    
    return None

def distanceBtwLandmarks(ldmk1, ldmk2):
    # each landmark object is a tuple of form:
    # (0, x: 0.604244828
    # y: 0.413355529
    # z: -0.458691299
    # visibility: 0.999419332
    # )
    point1 = ldmk1[1] 
    point2 = ldmk2[1]

    return ( (point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2 ) ** 0.5


while True:
    run(0)
    if cv2.waitKey(20) & 0XFF==ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break