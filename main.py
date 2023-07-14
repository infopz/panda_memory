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

cube_values = [1,1,2,2,3,3,4,4,5,5]

vision_method = "SIFT"  # can be SIFT, TM or NN


class Cube:
    def __init__(self, x, y, i):
        self.pos = [x, y]
        self.generate_pose_traj()
        self.i = i # TODO: rimuovere

        # 0 not_viewed, 1 viewed, 2 used
        self.status = 0
        self.description = None

    def generate_pose_traj(self):
        self.above_pose = control.generate_pose(self.pos[0], self.pos[1], 0.5+table_height, pi, 0, -pi / 4)
        self.grasp_pose = control.generate_pose(self.pos[0], self.pos[1], 0.13+table_height, pi, 0, -pi / 4)

        self.pick_traj = PandaTrajectory()
        self.pick_traj.add_pose(self.above_pose)
        self.pick_traj.add_pose(self.grasp_pose)

    def pick(self):
        # Move the EE from an arbitrary position to above the cube and then grasp it

        self.pick_traj.execute()
        time.sleep(1)
        control.close_gripper()
        time.sleep(1)

        PandaTrajectory(self.above_pose).execute()

    def view(self):
        # Assuming it's already picked, move the cube to the camera

        #PandaTrajectory(view_pose).execute()
        control.joint_move_to(human_view_conf)
        time.sleep(1)
        control.joint_move_to(view_conf)
        d = vision.get_descriptor(vision_method)
        # TODO: rimettere a posto
        time.sleep(1)
        self.description = cube_values[self.i]


        self.status = 1
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
            #PandaTrajectory(discard_red).execute()
            control.joint_move_to(random_shift(discard_red_conf))
        else:
            # PandaTrajectory(discard_pose).execute()
            control.joint_move_to(random_shift(discard_yellow_conf))
        time.sleep(0.5)
        control.open_gripper()
        self.status = 2
        PandaTrajectory(start_pose).execute()

    def pick_view_place(self):
        self.pick()
        self.view()
        self.place()

    def put_away(self, bin):
        self.pick()
        self.discard(bin)


def init_node():
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
    control.open_gripper()

    positions = insert_model.insert_many()
    cubes = [Cube(p[0], p[1], i) for i,p in enumerate(positions)] # TODO: modificare

    insert_model.insert_bins()

    # Prepare MoveIt
    control.add_scene_box("bin1", (0.25, 0.25, 0.2), (-0.5, 0.3, 0.1+table_height))
    control.add_scene_box("bin2", (0.25, 0.25, 0.2), (-0.5, -0.3, 0.1+table_height))
    control.add_scene_box("ground", (3, 3, 0.05), (0, 0, -0.025+table_height))


def random_shift(joint_conf):
    j = list(joint_conf)
    j[3] += random.uniform(-5,5) * 0.01745
    j[4] += random.uniform(-5,5) * 0.01745
    j[5] += random.uniform(-5, 5) * 0.01745
    return tuple(j)


def random_pick():
    global cubes
    cubes[3].pick_view_place()
    #cubes[3].put_away()


def test_grip():
    init_node()

    start_pose = control.generate_pose(0.4, 0, 0.6, pi, 0, -pi / 4)
    above_cube = control.generate_pose(0.4, -0.5, 0.5, pi, 0, -pi / 4)
    on_cube = control.generate_pose(0.4, -0.5, 0.13, pi, 0, -pi / 4)
    sopra = control.generate_pose(0.4, -0.5, 0.3, 0, 0, -pi / 4)
    sopra_box = control.generate_pose(-0.6, 0, 0.7, pi, 0, -pi / 4)

    # Porto nella conf iniziale
    control.move_to_position(start_pose)
    control.open_gripper()
    time.sleep(1)
    # insert_box.insert_box()
    insert_model.insert_box_sdf()
    insert_model.insert_bin()
    control.add_scene_box("bin", (0.5, 0.5, 0.4), (-0.7, 0, 0))
    time.sleep(2)

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
    control.move_to_position(sopra_box)
    control.open_gripper()
    # open_gripper()

def test():
    init_node()

    #start_pose = control.generate_pose(0.4, 0, 0.6 + table_height, pi, 0, -pi / 4)
    #view_pose = control.generate_pose(0.4, 0, 0.6 + 0.2 + table_height, 0, 0, -pi / 4)
    human_pose = control.generate_pose(0.5, 0, 0.6+table_height, -pi/4, pi/2, 0)

    h_conf = (-0.19195, -0.0698, 0.1396, -2.21615, -0.1745, 3.7343, -2.1987)
    new_h = (2.4779, 0.349, -2.792, -1.48325,-2.4779,3.68195,0.29665)
    h3 = (0.4886, -0.5933, -0.5235, -2.11145, 2.87925, 3.0363, -0.78525)


    #control.joint_move_to(start_pose)
    control.move_to_position(start_pose)
    time.sleep(1)
    #control.move_to_position(human_pose)

    control.joint_move_to(h3)

    time.sleep(1)
    control.joint_move_to(view_conf)

def start():
    init_node()
    prepare_world()

    t = threading.Thread(target=vision.start_camera_node)
    t.start()


    #random_pick()
    m = Memory(cubes, lambda x,y: x==y)
    #compare_func = vision.get_compare_func(vision_method)
    # TODO: rimettere a posto
    #m = Memory(cubes, compare_func)
    m.play()

if __name__ == "__main__":
    start()
