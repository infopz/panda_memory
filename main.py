import insert_box
import control
from control import PandaTrajectory

import moveit_commander
import rospy
import moveit_msgs
from geometry_msgs.msg import Pose

from math import pi
import time
import sys
import random

#  roslaunch panda_moveit_config demo_gazebo.launch

cubes = []
start_pose = control.generate_pose(0.4, 0, 0.6, pi, 0, -pi / 4)
view_pose = control.generate_pose(0.4, 0, 0.6, 0, 0, -pi / 4)

class Cube:
    def __init__(self, x, y):
        self.pos = [x, y]

        # 0 not_viewed, 1 viewed, 2 used
        self.status = 0
        self.description = None

    def pick(self):
        above_pose = control.generate_pose(self.pos[0], self.pos[1], 0.5, pi, 0, -pi / 4)
        grasp_pose = control.generate_pose(self.pos[0], self.pos[1], 0.13, pi, 0, -pi / 4)

        pick_traj = PandaTrajectory()
        pick_traj.add_pose(above_pose)
        pick_traj.add_pose(grasp_pose)

        pick_traj.execute()
        time.sleep(1)
        control.close_gripper()
        time.sleep(1)

        view_traj = PandaTrajectory()
        view_traj.add_pose(above_pose)
        view_traj.add_pose(view_pose)

        view_traj.execute()
        time.sleep(3)

        pick_traj.execute()
        time.sleep(1)
        control.open_gripper()


def init_node():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('python_control_node', anonymous=True)

    control.robot = moveit_commander.RobotCommander()
    control.move_group = moveit_commander.MoveGroupCommander("panda_arm")
    control.traj_pub = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    print "Python Control Node Started"


def load_cubes():
    # Insert cubes into gazebo and store their positions
    global cubes
    positions = insert_box.insert_many()
    cubes = [Cube(p[0], p[1]) for p in positions]


def random_pick():
    global cubes
    cubes[0].pick()


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
control.move_to_position(start_pose)
control.open_gripper()
load_cubes()
time.sleep(3)
random_pick()
