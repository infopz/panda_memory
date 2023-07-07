import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel
from geometry_msgs.msg import Pose


def insert_box():

	#rospy.init_node('box_inserter', anonymous=True)

	# Create a service client to set the model state
	#set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
	
	model_xml = open("box_ven.urdf", 'r').read()
	
	pos = Pose()
	pos.position.x = 0.4
	pos.position.y = -0.5
	pos.position.z = 0
	pos.orientation.w = 1
	
	
	spawn_model_client(model_name="colored_box", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos, reference_frame="world")


def insert_box_sdf():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

	model_xml = open("boxes/mystone.sdf", 'r').read()

	pos = Pose()
	pos.position.x = 0.4
	pos.position.y = -0.5
	pos.position.z = 0
	pos.orientation.w = 1

	spawn_model_client(model_name="box1", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")

def insert_stone():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

	model_xml = open("/home/infopz/memoryws/src/franka_ros/franka_gazebo/models/stone/model.sdf", 'r').read()

	pos = Pose()
	pos.position.x = 0.4
	pos.position.y = -0.5
	pos.position.z = 0
	pos.orientation.w = 1

	spawn_model_client(model_name="stone", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")


def insert_mystone():
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

	model_xml = open("mystone.urdf", 'r').read()

	pos = Pose()
	pos.position.x = 0
	pos.position.y = -0.2
	pos.position.z = 1
	pos.orientation.w = 1

	spawn_model_client(model_name="box1", model_xml=model_xml, robot_namespace='/foo', initial_pose=pos,
					   reference_frame="world")


if __name__ == '__main__':
	insert_box()

