from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
	
	#Define filenames
	pkg_bot = 'tb3_waffle_description'
	urdf_file='robot_waffle.urdf.xacro'
	rviz_file = 'waffle.rviz'

	#Set the paths
	pkg_descripton = FindPackageShare(pkg_bot)

	urdf_path=PathJoinSubstitution([pkg_descripton,'urdf',urdf_file])

	rviz_config_file=PathJoinSubstitution([pkg_descripton,'rviz',rviz_file])

	robot_description=ParameterValue(Command([
		'xacro',' ',urdf_path]),
		value_type=str)
	
	# Subscribe to the joint states of the robot, and publish the 3D pose of each link.
	robot_state_publisher_bot=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_bot',
		output='screen',
		parameters=[{
			'robot_description':robot_description}]
	)
	
	# Publishes the joint states from URDF file 
	joint_state_publisher_gui_bot=Node(
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		name='joint_state_publisher_gui_bot',
		output='screen'		
	)

	#RViz node 
	rviz_bot=Node(
		 executable ='rviz2',
		 name       ='rviz2_bot',
		 output     ='log',
		 arguments  =['-d', rviz_config_file])



	return LaunchDescription([robot_state_publisher_bot, joint_state_publisher_gui_bot, rviz_bot ])