import random

import insert_model
from insert_model import table_height
import control
import vision
from control import PandaTrajectory
from memory import Memory

import moveit_commander
import rospy
import moveit_msgs

from math import pi
import time
import sys
import threading

#  roslaunch panda_moveit_config demo_gazebo.launch

cubes = []
start_pose = control.generate_pose(0.4, 0, 0.6+table_height, pi, 0, -pi / 4)
view_pose = control.generate_pose(0.4, 0, 0.6+0.2+table_height, 0, 0, -pi / 4)
discard_pose = control.generate_pose(-0.5, 0.3, 0.6+table_height, pi, 0, -pi / 4)
discard_red = control.generate_pose(-0.5, -0.3, 0.6+table_height, pi, 0, -pi / 4)

view_conf = (0.701450851863969, -0.5962195187508552, -0.5182710831694077, -2.1604622324191816, 2.858713059098573, 1.5169423362307946, -1.0614675683124668)
discard_yellow_conf = (0.47683751216862813, -1.109671424430939, 1.125051316972466, -1.441047919094805, 1.119399674382163, 1.1142596839421959, 2.749975579652851)
discard_red_conf = (-0.20707629270604944, -1.4621954014427345, -0.7580910829375247, -1.4170987058591002, -1.5750853554134814, 0.7520932299359213, -1.101073174579394)
human_view_conf = (0.4886, -0.5933, -0.5235, -2.11145, 2.87925, 3.0363, -0.78525)

cube_values = [1,1,2,2,3,3,4,4,5,5]  # rimuovere

vision_method = "SIFT"  # can be SIFT, TM or NN


class Cube:
    # Define a cube with its position and state
    # The method defines how to move it
    def __init__(self, x, y, i):
        self.pos = [x, y]
        self.generate_pose_traj()
        self.i = i # TODO: rimuovere

        # 0 not_viewed, 1 viewed, 2 used
        self.status = 0
        self.description = None

    def generate_pose_traj(self):
        # Generates some variables that will be used later

        self.above_pose = control.generate_pose(self.pos[0], self.pos[1], 0.5+table_height, pi, 0, -pi / 4)
        self.grasp_pose = control.generate_pose(self.pos[0], self.pos[1], 0.13+table_height, pi, 0, -pi / 4)

        self.pick_traj = PandaTrajectory()
        self.pick_traj.add_pose(self.above_pose)
        self.pick_traj.add_pose(self.grasp_pose)

    def pick(self):
        # Move the robot from an arbitrary position to above the cube and then grasp it

        self.pick_traj.execute()
        time.sleep(1)
        control.close_gripper()
        time.sleep(1)

        PandaTrajectory(self.above_pose).execute()

    def view(self):
        # Assuming it's already picked, first move the cube in front of the human
        # and then to the camera. At the end go back above the original position

        control.joint_move_to(human_view_conf)
        time.sleep(1)

        control.joint_move_to(view_conf)

        self.description = vision.get_descriptor(vision_method)
        time.sleep(0.5)

        # TODO: rimettere a posto
        #time.sleep(1)
        #self.description = cube_values[self.i]

        self.status = 1  # viewed status

        PandaTrajectory(self.above_pose).execute()

    def place(self):
        # Place the cube to its original position

        self.pick_traj.execute()
        time.sleep(1)
        control.open_gripper()

        # Return to the initial position
        p = PandaTrajectory()
        p.add_pose(self.above_pose)
        p.add_pose(start_pose) # TODO: magari cambiare con una con i joint
        p.execute()

    def discard(self, bin):
        # Discard the cube to the box behind and return to the initial position

        if bin == 0:
            control.joint_move_to(random_shift(discard_red_conf))
        else:
            control.joint_move_to(random_shift(discard_yellow_conf))

        time.sleep(0.5)
        control.open_gripper()

        self.status = 2  # discared status

        PandaTrajectory(start_pose).execute()

    def pick_view_place(self):
        self.pick()
        self.view()
        self.place()

    def put_away(self, bin):
        self.pick()
        self.discard(bin)


def init_node():
    # Initialize the ROS node and the commander to control the robot, the gripper and the scene in moveit

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('python_control_node', anonymous=True)

    control.robot = moveit_commander.RobotCommander()
    control.move_group = moveit_commander.MoveGroupCommander("panda_arm")
    control.scene = moveit_commander.PlanningSceneInterface()
    control.traj_pub = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    print "Python Control Node Started"


def prepare_world():
    # Insert all the necessary models on Gazebo and MoveIt. Stores the cubes position
    global cubes

    # Prepare Gazebo and adjust robot
    insert_model.insert_table()
    insert_model.insert_camera()

    time.sleep(0.5)

    control.joint_move_to([0, -pi / 4, 0, -pi / 2, 0, pi / 3, 0])
    control.move_to_position(start_pose)
    control.open_gripper()

    positions = insert_model.insert_boxes()
    cubes = [Cube(p[0], p[1], i) for i,p in enumerate(positions)] # TODO: modificare

    insert_model.insert_bins()

    # Prepare MoveIt scene
    control.add_scene_box("bin1", (0.25, 0.25, 0.2), (-0.5, 0.3, 0.1 + table_height))
    control.add_scene_box("bin2", (0.25, 0.25, 0.2), (-0.5, -0.3, 0.1 + table_height))
    control.add_scene_box("table", (1.5, 1.5, 0.05), (-0.1, 0, -0.025 + table_height))
    control.add_scene_box("human", (0.05, 3, 2), (0.65+0.025, 0, 1 + table_height))
    control.add_scene_box("camera", (0.1, 0.1, 0.1), (0.4, 0, 1.2 + table_height))


def random_shift(joint_conf):
    # Randomize the value of the joints 3-5 by 5 degree
    # Slightly changes the EE pose in the workspace

    j = list(joint_conf)
    j[3] += random.uniform(-5, 5) * 0.01745
    j[4] += random.uniform(-5, 5) * 0.01745
    j[5] += random.uniform(-5, 5) * 0.01745
    return tuple(j)


def start():
    init_node()
    prepare_world()

    # Starts the camera node that runs independently
    t = threading.Thread(target=vision.start_camera_node)
    t.start()

    #m = Memory(cubes, lambda x,y: x==y)
    compare_func = vision.get_compare_func(vision_method)
    # TODO: rimettere a posto
    m = Memory(cubes, compare_func)
    m.play()


if __name__ == "__main__":
    start()
