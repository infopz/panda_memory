import insert_box
import control
from control import PandaTrajectory

import moveit_commander
import rospy
import moveit_msgs

from math import pi
import time
import sys


#  roslaunch panda_moveit_config demo_gazebo.launch

def init_node():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('python_control_node', anonymous=True)

    control.robot = moveit_commander.RobotCommander()
    control.move_group = moveit_commander.MoveGroupCommander("panda_arm")
    control.traj_pub = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    print "Python Control Node Started"

def test_grip():
    start_pose = control.generate_pose(0.4, 0, 0.6, pi, 0, -pi / 4)
    above_cube = control.generate_pose(0.4, -0.5, 0.5, pi, 0, -pi / 4)
    on_cube = control.generate_pose(0.4, -0.5, 0.13, pi, 0, -pi / 4)
    sopra = control.generate_pose(0.4, -0.5, 0.3, 0, 0, -pi / 4)

    # Porto nella conf iniziale
    control.move_to_position(start_pose)
    control.open_gripper()
    # insert_box.insert_box()
    insert_box.insert_box_sdf()
    time.sleep(3)

    tocube = PandaTrajectory()
    tocube.add_pose(above_cube)
    tocube.add_pose(on_cube)
    tocube.execute()

    time.sleep(1)
    control.close_gripper()
    time.sleep(3)

    # muovicubo = PandaTrajectory()
    # muovicubo.add_pose(difianco)
    # muovicubo.execute()
    control.move_to_position(above_cube)
    # move_to_position(start_pose)
    time.sleep(4)
    control.move_to_position(sopra)
    time.sleep(4)
    control.move_to_position(above_cube)
    control.open_gripper()
    # open_gripper()


init_node()
test_grip()
