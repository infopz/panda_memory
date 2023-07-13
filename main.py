import insert_model
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
start_pose = control.generate_pose(0.4, 0, 0.6, pi, 0, -pi / 4)
view_pose = control.generate_pose(0.4, 0, 0.6, 0, 0, -pi / 4)
discard_pose = control.generate_pose(-0.6, 0, 0.7, pi, 0, -pi / 4)

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
        self.above_pose = control.generate_pose(self.pos[0], self.pos[1], 0.5, pi, 0, -pi / 4)
        self.grasp_pose = control.generate_pose(self.pos[0], self.pos[1], 0.13, pi, 0, -pi / 4)

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

        PandaTrajectory(view_pose).execute()
        self.description = vision.get_descriptor(vision_method)

        #time.sleep(3)
        #self.description = cube_values[self.i]

        self.status = 1
        PandaTrajectory(self.above_pose).execute()

    def place(self):
        # Place the cube to its original position

        self.pick_traj.execute()
        time.sleep(1)
        control.open_gripper()
        PandaTrajectory(self.above_pose).execute()

    def discard(self):
        # Discard the cube to the box behind and return to the initial position

        PandaTrajectory(discard_pose).execute()
        time.sleep(1)
        control.open_gripper()
        self.status = 2
        PandaTrajectory(start_pose).execute()

    def pick_view_place(self):
        self.pick()
        self.view()
        self.place()

    def put_away(self):
        self.pick()
        self.discard()


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
    # Insert cubes into gazebo and store their positions
    global cubes

    insert_model.insert_bin()
    control.add_scene_box("bin", (0.5, 0.5, 0.4), (-0.7, 0, 0))

    insert_model.insert_camera()

    time.sleep(0.5)

    control.joint_move_to([0, -pi / 4, 0, -pi / 2, 0, pi / 3, 0])
    control.open_gripper()

    positions = insert_model.insert_many()
    cubes = [Cube(p[0], p[1], i) for i,p in enumerate(positions)] # TODO: modificare



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

def start():
    init_node()
    prepare_world()

    t = threading.Thread(target=vision.start_camera_node)
    t.start()

    #random_pick()
    #m = Memory(cubes, lambda x,y: x==y)
    compare_func = vision.get_compare_func(vision_method)

    m = Memory(cubes, compare_func)
    m.play()

if __name__ == "__main__":
    start()
