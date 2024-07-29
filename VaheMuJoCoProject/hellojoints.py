import mujoco
import cv2
import os
from UtilityClasses import helloUtility as util

#xml = open("VaheMuJoCoProject\\hellojoints.xml", "r").read()

# quick note:
# when using x and y axes for direction, the direction your camera faces is the cross product 

xml = """
<mujoco>
    <option gravity = "0 0 -1000" integrator = "RK4"/>

    <asset>
        <mesh file = "VaheMuJoCoProject\\STLFiles\\CameraMount.stl" />
        <texture name = "grid" type = "2d" builtin="checker" rgb1 = "0 0 1" 
        rgb2 = "1 1 1" width = "300" height = "300"/>
        <material name = "grid" texture = "grid" texrepeat = "8 8" reflectance = "0.2"/>
    </asset>

    <worldbody>

        {lights}


        <camera name="camera1" euler = "-45 0 180" pos = "0 20 20"/>
        <camera name = "camera2" pos = "0 -20 4" xyaxes = "1 0 0 0 0 1"/>
        <geom name = "floor" type = "plane" pos = "0 0 -1" size = "20 20 0.1" material = "grid"/>

        <body name = "ball box system" euler = "0 0 0">

            <freejoint/>
            <geom name="ball" type="sphere" mass = "900" size="0.5" rgba = "1 1 1 1" pos = "0 0 5" />
            <geom name="box" type="box" mass = "900" size="0.5 0.5 0.5" rgba = "1 1 1 1" pos = "0 5 4" />

        </body>

        <geom type = "mesh" mesh = "CameraMount" pos = "0 0 0" />

    </worldbody>

    <keyframe>
        <key name = "spinning"  qpos = "0 0 0 0 0 0 0" qvel = "0 0 0 0 0 200"/>
    </keyframe>

</mujoco>


""".format(lights = util.ringOfLightsXMLString(1, 5, 20))


model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)


mujoco.mj_kinematics(model, data)
mujoco.mj_forward(model, data)

scene_optioner = mujoco.MjvOption()
#scene_optioner.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True



FPS = 100

mujoco.mj_resetDataKeyframe(model, data, 0)

with mujoco.Renderer(model) as renderer:
    while True:

        mujoco.mj_step(model, data)
        
        renderer.update_scene(data, scene_option=scene_optioner, camera="camera1")

        frame = renderer.render()
        cv2.imshow("WINDOW1", util.resizeImage(frame, 2))


        renderer.update_scene(data, scene_option=scene_optioner, camera="camera2")

        frame = renderer.render()
        cv2.imshow("WINDOW2", util.resizeImage(frame, 2))


        if cv2.waitKey(int(1000/FPS) - 2) & 0XFF==ord('q'):
            break


cv2.destroyAllWindows()
exit()
