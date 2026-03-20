from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,PathJoinSubstitution,LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name ='tb3_waffle_description'
    urdf_file='robot_waffle.urdf.xacro'
    rviz_file='waffle.rviz'

    pkg_description=FindPackageShare(pkg_name) 
    #Define world file path  
    house_description = FindPackageShare('world_display')
    
    world_file_name = 'house_world.sdf'
    world = PathJoinSubstitution([house_description, 'worlds', world_file_name])
    
    #Declare Gazebo launch arguments
    declare_gz_args_cmd = DeclareLaunchArgument(
          name = 'gz_args',
          default_value = ['-r -v 4 ',world],
          description = 'Defining the world for robot model'
     )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Uses simulated clock when set to true'
    )

    #Setting gz_resource path
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[PathJoinSubstitution([house_description, 'demo/models'])]
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Initialization of directories
    xacro_file_GZ =PathJoinSubstitution([pkg_description,'urdf',urdf_file]) 
    #Robot description
    robot_description=ParameterValue(Command([
          'xacro',' ',xacro_file_GZ]),
          value_type=str)
    
    #Define launch arguments for intial pose
    x_args =  DeclareLaunchArgument('x', default_value='0.0', description='Initial X position of the robot')
    y_args =  DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position of the robot')
    z_args =  DeclareLaunchArgument('z', default_value='0.0', description='Initial Z position of the robot')
    roll_args =  DeclareLaunchArgument('R', default_value='0.0', description='Initial Roll orientation of the robot')
    pitch_args =  DeclareLaunchArgument('P', default_value='0.0', description='Initial Pitch orientation of the robot')
    yaw_args =  DeclareLaunchArgument('Y', default_value='0.0', description='Initial Yaw orientation of the robot')

    #Launch configuration variables
    gz_args = LaunchConfiguration('gz_args')

    x=LaunchConfiguration('x')
    y=LaunchConfiguration('y')
    z=LaunchConfiguration('z')
    R=LaunchConfiguration('R')
    P=LaunchConfiguration('P')
    Y=LaunchConfiguration('Y')

    #Combined all together creating the gazebo node
    gazebo_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'])
               ]),
                launch_arguments={'gz_args': gz_args}.items())
    
    #Spawn the robot
    spawn_entity_robot=Node(
        package='ros_gz_sim',
        executable='create',
        arguments   = ['-topic', 'robot_description' , '-name', 'turtlebot3_waffle',
                       'x', x, 'y', y, 'z', z,
                       'R', R, 'P', P, 'Y', Y,'-z', '0.5'],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Bridge
    bridge_param=PathJoinSubstitution([FindPackageShare('tb3_waffle_description'), 'config', 'gz_bridge.yaml'])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_param,
            'use_sim_time': use_sim_time,
            }],
        output='screen'
    )


    # Robot State Publisher to generate the /robot state topic with URDF data
    robot_state_publisher_bot=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_bot',
		output='screen',
		parameters=[{
            'use_sim_time': use_sim_time,
			'robot_description':robot_description}]
     )
    
    ekf_launch = Node (
        package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{os.path.join(get_package_share_directory("tb3_waffle_description"), 'config', 'ekf.yaml')},
                        {'use_sim_time': True,
                         }]         
    )
    
    return LaunchDescription([
        declare_gz_args_cmd,
        declare_use_sim_time_cmd,
        set_gz_resource,
        x_args,y_args,z_args,roll_args,pitch_args,yaw_args,
        bridge,
        gazebo_launch,
        TimerAction(
            period=30.0,
            actions=[
                        robot_state_publisher_bot,
                        spawn_entity_robot
            ]
        ),
        ekf_launch
    ])