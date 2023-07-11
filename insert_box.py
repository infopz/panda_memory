import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel
from geometry_msgs.msg import Pose
import random

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
	pos.position.z = 0
	pos.orientation.w = 1

	spawn_model_client(model_name="big_bin", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")


def insert_many():
	types = ["burger", "butterfly", "cat"]

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	types = types + types
	random.shuffle(types)

	positions = []

	for i in range(len(types)):
		#model_xml = open("./boxes/box_"+types[i]+".urdf", 'r').read()
		model_xml = open("boxes/box_burger.sdf", 'r').read()
		pos = Pose()
		pos.position.x = 0.3 + 0.15*(i%2)
		pos.position.y = -0.3 + 0.15*(i//2)
		pos.position.z = 0.0
		pos.orientation.w= 1
		spawn_model_client(model_name="box"+str(i), model_xml=model_xml, robot_namespace='/foo', initial_pose=pos, reference_frame="world")
		positions.append((pos.position.x, pos.position.y))

	return positions


if __name__ == '__main__':
	insert_box_urdf()

