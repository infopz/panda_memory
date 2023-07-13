import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel
from geometry_msgs.msg import Pose
import random

table_height = 0.83

def insert_box_urdf():

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
	
	model_xml = open("/home/infopz/Desktop/colored_box.urdf", 'r').read()
	
	pos = Pose()
	pos.position.x = 0.4
	pos.position.y = -0.5
	pos.position.z = 0
	pos.orientation.w = 1

	spawn_model_client(model_name="colored_box", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos, reference_frame="world")


def insert_box_sdf():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

	model_xml = open("boxes/box_burger.sdf", 'r').read()

	pos = Pose()
	pos.position.x = 0.4
	pos.position.y = -0.5
	pos.position.z = 0
	pos.orientation.w = 1

	spawn_model_client(model_name="box1", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")


def insert_bin():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

	model_xml = open("boxes/bin.sdf", 'r').read()

	pos = Pose()
	pos.position.x = -0.7
	pos.position.y = 0
	pos.position.z = 0+table_height
	pos.orientation.w = 1

	spawn_model_client(model_name="big_bin", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")


def insert_bins():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

	bin_yellow = open("boxes/bin_yellow.sdf", 'r').read()
	bin_red = open("boxes/bin_red.sdf", "r").read()

	pos = Pose()
	pos.position.x = -0.5
	pos.position.y = 0.3
	pos.position.z = 0 + table_height
	pos.orientation.w = 1

	spawn_model_client(model_name="bin_yellow", model_xml=bin_yellow, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")

	pos.position.y = -0.3

	spawn_model_client(model_name="bin_red", model_xml=bin_red, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")

def insert_many():
	types = ["burger", "butterfly", "cat", "car", "tree"]

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	types = types + types
	random.shuffle(types)

	positions = []

	for i in range(len(types)):
		model_xml = open("boxes/colored/box_"+types[i]+".sdf", 'r').read()
		#model_xml = open("boxes/box_burger.sdf", 'r').read()
		pos = Pose()
		pos.position.x = 0.4 + 0.15*(i%2)
		pos.position.y = -0.3 + 0.15*(i//2)
		pos.position.z = 0+table_height
		pos.orientation.w= 1
		spawn_model_client(model_name="box"+str(i), model_xml=model_xml, robot_namespace='/foo', initial_pose=pos, reference_frame="world")
		positions.append((pos.position.x, pos.position.y))

	return positions


def insert_camera():

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	model_xml = open("boxes/camera_model.sdf", 'r').read()

	pos = Pose()
	pos.position.x = 0.4
	pos.position.y = 0
	pos.position.z = 1.2+table_height
	pos.orientation.w = 0.5353686

	spawn_model_client(model_name="camera", model_xml=model_xml, initial_pose=pos, reference_frame="world")

def insert_table():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	model_xml = open("boxes/table_new.sdf", 'r').read()

	pos = Pose()
	pos.position.x = -0.1
	pos.position.y = 0
	pos.position.z = 0
	pos.orientation.w = 1

	spawn_model_client(model_name="table", model_xml=model_xml, initial_pose=pos, reference_frame="world")


if __name__ == '__main__':
	insert_box_urdf()

