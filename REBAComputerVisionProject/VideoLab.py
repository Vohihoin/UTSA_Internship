import os

import cv2
import mediapipe as mp
from ultralytics import YOLO
import numpy as np
import math

# Shrink image method
def shrinkImage(image, factor):
    height = image.shape[0]
    width = image.shape[1]

    newHeight = math.floor(height / (factor ** (0.5)))
    newWidth = math.floor(width / (factor ** (0.5)))

    return ( cv2.resize(image, (newWidth, newHeight)) )


source = os.path.join("UTSA_Internship","UTSA AI VIDEOS","DJI_0521.MP4")
webcam = cv2.VideoCapture(source)
model = YOLO('yolov8n.pt')
ret = True

while ret:
    
    ret, frame = webcam.read()

    results = model.track(frame, persist=True)
    
    
    if len(results[0].boxes) > 0:

        for box in results[0].boxes:
        
            box_ = box.xyxy.numpy().tolist()[0]
            x1 = int(box_[0])
            y1 = int(box_[1])
            x2 = int(box_[2])
            y2 = int(box_[3])


            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 1)

    #processedFrame = results[0].plot()
    cv2.imshow("Window", shrinkImage(frame, 4))

    if cv2.waitKey(10) & 0XFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()