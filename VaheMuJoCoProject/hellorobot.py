import mujoco
import cv2


xml = """
<mujoco>

    <asset>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\base.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\big_connector_rod_back.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\big_connector_rod_front.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\big_connector_rod_top.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\long_hinge_left.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\long_hinge_right.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\short_hinge_left.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\short_hinge_right.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\small_connector_rod.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\top_board.stl"/>
    </asset>

    <worldbody>
        <camera name="main_camera" pos = "0 10 10" mode="targetbody" target="robot"/>
        <geom name = "floor" type = "plane" size = "10 10 0.1" pos = "0 0 0" rgba = "1 1 1 1"/>
        <body name="robot" euler = "0 0 0" pos = "0 0 0">
            <geom name="base" type="mesh" mesh = "base" pos = "0 0 0" rgba = "0 0 1 1"/>
        </body>

    </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

mujoco.mj_kinematics(model, data)
mujoco.mj_forward(model, data)

with mujoco.Renderer(model, 480, 640) as renderer:

    mujoco.mj_resetData(model, data)

    while True:
        
        mujoco.mj_step(model, data)
        renderer.update_scene(data, camera="main_camera")

        frame = renderer.render()
        cv2.imshow("WINDOW", frame)
        if cv2.waitKey(20) & 0XFF==ord('q'):

            break

cv2.destroyAllWindows()
exit()
