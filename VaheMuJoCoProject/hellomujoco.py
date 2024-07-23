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

xml = """
<mujoco>
     <worldbody>
          <light name = "light1" pos = "0 0 1" dir = "{}" active = "true"/>
          <geom name = "geom1" type = "box" pos = "1 1 1" size = "0.2 0.2 0.2" rgba="0 1 0 1" />
          <geom name = "geom2" type = "sphere" pos = "0.5 0.5 0.5" size = "0.2" rgba = "0 0 1 1" />
     </worldbody>
</mujoco>


""".format("0 0 -1")

duration = 10 # seconds
framerate = 60 # Hz


model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_kinematics(model, data) # basically propagating co-ordinates
mujoco.mj_forward(model, data) # basically propagating visualisation

frames = []
FPS = 60
with mujoco.Renderer(model) as renderer:
    
    mujoco.mj_resetData(model, data)
    while data.time < duration:
        
        mujoco.mj_step(model, data) # basically our 2ms timer
        cv2.waitKey(int(1000/FPS) - 2)

        if len(frames) < data.time * framerate:
            
          renderer.update_scene(data)
          frame = renderer.render()
          cv2.imshow("Window", resizeImage(frame, 2))


