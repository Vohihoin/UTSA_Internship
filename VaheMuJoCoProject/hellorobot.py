import mujoco
import cv2
from UtilityClasses import helloUtility as util


xml = """
<mujoco>

    <option gravity = "0 -10 0"/>

    <asset>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\base.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\base2.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\big_connector_rod_back.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\big_connector_rod_front.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\big_connector_rod_top.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\long_hinge_left.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\long_hinge_right.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\short_hinge_left.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\short_hinge_right.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\small_connector_rod.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\top_board.stl"/>
        <mesh file="VaheMuJoCoProject\\STLFiles\\BlackTeamRobot\\top_board2.stl"/>
    </asset>




    <worldbody>
        <camera name="main_camera" pos = "0.2 1 0.75" mode="targetbody" target="robot"/>
        {lights}
        <body name="robot" euler = "0 0 0" pos = "0 0 0">

            <geom name="base" type="mesh" mesh = "base" pos = "0 0 0" euler = "-90 0 180"/>
            <geom name="big_connector_rod_back" type="mesh" mesh="big_connector_rod_back" euler = "90 0 0" pos = "0 0.012700 0"/>


            <body name="top_board" pos = "0 -0.050800 0">
                <joint name= "board_to_base" pos="0 0 0" axis = "0 1 0" frictionloss="0"/>
                <geom name="top_board" euler = "-90 0 0" type="mesh" mass = "1" mesh="top_board" />


            </body>
            
        </body>

        <body name="base2"  pos = "0.254000 0 0">

            <geom name="base2" type="mesh" mesh = "base2" euler = "-90 0 180"/>
            <body name = "short_hinge_left" pos = "0 -0.044450 0">

                <joint name="short_hinge_to_base2" pos = "0 0 0" axis="0 1 0" frictionloss="0"/>
                        
                <geom name = "short_hinge_left" type = "mesh" euler = "-90 0 0" mesh = "short_hinge_left" />

                <body name = "short_hinge_right" pos = "0 -0.273050 0">
                    <geom name = "short_hinge_right" type = "mesh" euler = "-90 0 0" mesh = "short_hinge_right" />
                </body>

                <body name="long_hinge_left" pos = "0.050800 -0.006350 0">
                    <joint name="long_hinge_to_short" pos="0 0 0" axis="0 1 0" frictionloss="0"/>
                    <geom name= "long_hinge_left" type="mesh" mesh = "long_hinge_left"/>

                    <body name="long_hinge_right" pos = "0 -0.257581 0">
                        <geom name= "long_hinge_right" type="mesh" mesh = "long_hinge_right"/>
                    </body>

                    <body name="top_board2" pos = "-0.101600 0 0">
                        <geom name="top_board2" euler = "-90 0 0" type="mesh" mesh="top_board2" />
                    </body>

                </body>

            </body>
        </body>


    </worldbody>


    <equality>
        <weld body1="top_board2" body2="top_board" relpose="0.304800 0 0 -0.7071068 0 0 0.7071068"/>
    </equality>

    <actuator>
        <motor name = "board_to_base_motor" joint="board_to_base"/>
        <motor name = "short_hinge_to_base2_motor" joint="short_hinge_to_base2"/>
        <motor name = "long_hinge_to_short_motor" joint="long_hinge_to_short"/>
    </actuator>



</mujoco>
""".format(lights = util.ringOfLightsXMLString(50, 3, 5))


"""
                <body name="long_hinge_left" pos = "0.304800 0 0">

                    <joint name="long_hinge_left_to_board" axis="0 1 0" frictionloss="0"/>
                    <geom name= "long_hinge_left" type="mesh" mesh = "long_hinge_left"/>

                    <body name="long_hinge_right" pos = "0 -0.257581 0">
                        <geom name= "long_hinge_right" type="mesh" mesh = "long_hinge_right"/>
                    </body>

                    <body name = "short_hinge_left" pos = "0.101600 0.006350 0">

                        <joint name="short_hinge_to_long_hinge" pos = "0 0 0" axis="0 1 0" frictionloss="0"/>
                        
                        <geom name = "short_hinge_left" type = "mesh" euler = "-90 0 0" mesh = "short_hinge_left" />

                        <body name = "short_hinge_right" pos = "0 -0.273050 0">
                            <geom name = "short_hinge_right" type = "mesh" euler = "-90 0 0" mesh = "short_hinge_right" />
                        </body>

                        <body name = "small_connector_rod" pos = "0 0.006350 0">
                            <geom name = "small_connector_rod" type = "mesh" euler = "90 0 0" mesh = "small_connector_rod" />
                        </body>


                    </body>

                </body>




"""


model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

mujoco.mj_kinematics(model, data)
mujoco.mj_forward(model, data)

scene_optioner = mujoco.MjvOption()
scene_optioner.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True


with mujoco.Renderer(model, 480, 640) as renderer:

    mujoco.mj_resetData(model, data)
    count = 1

    while True:

        data.eq_active = True
        
        mujoco.mj_step(model, data)
        renderer.update_scene(data, camera="main_camera", scene_option=scene_optioner)

        frame = renderer.render()

        if count == 1:
            cv2.imwrite("first_frame.png", frame)
            count += 1

        cv2.imshow("WINDOW", util.resizeImage(frame, 2))
        if cv2.waitKey(20) & 0XFF==ord('q'):

            break


cv2.destroyAllWindows()
exit()
