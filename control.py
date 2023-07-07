import moveit_msgs.msg
import geometry_msgs.msg
import franka_gripper.msg

from math import pi
from tf.transformations import quaternion_from_euler
import copy
import time
import actionlib

# Main script will fill this
robot = None
move_group = None
traj_pub = None


class PandaTrajectory:
    def __init__(self):
        self.waypoints = []
        self.computed_plan = None

    def get_pose(self, pos=-1):
        return self.waypoints[pos]

    def add_pose(self, pose, pos=-1):
        if pos == -1:
            self.waypoints.append(copy.deepcopy(pose))
        else:
            self.waypoints.insert(pos, copy.deepcopy(pose))

    def plan(self):
        (self.computed_plan, fraction) = move_group.compute_cartesian_path(
            self.waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

    def display(self, plan=True):
        if plan:
            self.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(self.computed_plan)
        # Publish
        traj_pub.publish(display_trajectory)

    def execute(self, plan=True):
        if plan:
            self.plan()
        move_group.execute(self.computed_plan, wait=True)


def get_info():
    # Print useful information

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    print "============ Available Planning Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()


def joint_move_to(q):
    # Bring robot in a desired configuration in joint space

    # rest [0,-pi/4,0,-pi/2,0.pi/3,0]

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = q[0]
    joint_goal[1] = q[1]
    joint_goal[2] = q[2]
    joint_goal[3] = q[3]
    joint_goal[4] = q[4]
    joint_goal[5] = q[5]
    joint_goal[6] = q[6]

    move_group.go(joint_goal, wait=True)

    move_group.stop()
    move_group.clear_pose_targets()


def move_to_position(pose):
    # Bring robot in a desired configuration in task space

    move_group.set_pose_target(pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


def open_gripper():

    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
    client.wait_for_server()

    goal = franka_gripper.msg.MoveGoal()
    goal.width = 0.08
    goal.speed = 0.1

    client.send_goal(goal)
    client.wait_for_result()
    r = client.get_result()

    if r.success:
        return True
    else:
        print r.error
        return r.error


def close_gripper():

    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client.wait_for_server()

    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.05
    goal.epsilon.inner = 0.04
    goal.epsilon.outer = 0.04
    goal.speed = 0.1
    goal.force = 10

    client.send_goal(goal)
    client.wait_for_result()
    r = client.get_result()

    if r.success:
        return True
    else:
        # se chiude senza niente da un errore, ma funziona lo stesso
        print r.error
        return r.error


def generate_pose(x, y, z, rx, py, yz):
    # Converts coordinates + RPY in Pose
    # x, y, z coordinted referred to world RF
    # rx, py, yz rotation angle (in radiants) as roll, pith and yaw on x,y,z

    pose = geometry_msgs.msg.Pose()

    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    quat = quaternion_from_euler(rx, py, yz)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose


if __name__ == "__main__":
    import insert_box

    print "Execution Started"

    start_pose = generate_pose(0.4, 0, 0.6, pi, 0, -pi/4)
    above_cube = generate_pose(0.4, -0.5, 0.5, pi, 0, -pi/4)
    on_cube = generate_pose(0.4, -0.5, 0.13, pi, 0, -pi/4)
    sopra = generate_pose(0.4, -0.5, 0.3, 0, 0, -pi/4)

    # Porto nella conf iniziale
    move_to_position(start_pose)
    open_gripper()
    #insert_box.insert_box()
    insert_box.insert_box_sdf()
    time.sleep(3)

    tocube = PandaTrajectory()
    tocube.add_pose(above_cube)
    tocube.add_pose(on_cube)
    tocube.execute()

    time.sleep(1)
    close_gripper()
    time.sleep(3)

    #muovicubo = PandaTrajectory()
    #muovicubo.add_pose(difianco)
    #muovicubo.execute()
    move_to_position(above_cube)
    #move_to_position(start_pose)
    time.sleep(4)
    move_to_position(sopra)
    time.sleep(4)
    move_to_position(above_cube)
    open_gripper()
    #open_gripper()

'''
def test_grip():
    start_pose = control.generate_pose(0.4, 0, 0.6, pi, 0, -pi / 4)
    above_cube = control.generate_pose(0.4, -0.5, 0.5, pi, 0, -pi / 4)
    on_cube = control.generate_pose(0.4, -0.5, 0.13, pi, 0, -pi / 4)
    sopra = control.generate_pose(0.4, -0.5, 0.3, 0, 0, -pi / 4)

    # Porto nella conf iniziale
    control.move_to_position(start_pose)
    control.open_gripper()
    # insert_box.insert_box()
    #insert_box.insert_box_sdf()
    insert_box.insert_many()
    time.sleep(3)
    raise Exception

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

'''