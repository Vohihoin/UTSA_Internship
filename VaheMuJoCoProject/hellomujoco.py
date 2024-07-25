import os
import mujoco
import time
import itertools
import numpy as np
import mediapy as media
import matplotlib.pyplot as plt
import cv2
from PIL import Image

def resizeImage(image, factor):
    
    h, w, c = image.shape
    return(cv2.resize(image, (int(w*factor), int(h*factor))))


print(os.path.isfile("hellomujoco.xml"))
xml = open("hellomujoco.xml", "r").read()

duration = 10 # seconds
framerate = 60 # Hz


model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_kinematics(model, data) # basically propagating co-ordinates
mujoco.mj_forward(model, data) # basically propagating visualisation

frames = []
FPS = 60
with mujoco.Renderer(model) as renderer:
    
    mujoco.mj_resetData(model, data) # resets all quantities to as they were at the beginning of the simulation

    while True:
        
        mujoco.mj_step(model, data) # moves the simulation forward in time waits 2 sec
        renderer.update_scene(data) # refresging rendering object
        frame = renderer.render()

        cv2.imshow("Window", resizeImage(frame, 2))

        if cv2.waitKey(int(1000/FPS) - 2) & 0XFF==ord('q'):
            break

    

cv2.destroyAllWindows()
exit()




