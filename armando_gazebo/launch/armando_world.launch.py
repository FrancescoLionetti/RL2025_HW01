import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    #URDF version
    '''#Load the robot description
    urdf_file_name = 'arm.urdf'
    
    # Build the absolute path to the URDF file inside the armando_description package
    urdf = os.path.join(
        get_package_share_directory('armando_description'), 'urdf',
        urdf_file_name
    )
    # Open and read the content of the URDF file
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    params = {"robot_description": robot_desc}'''

    #XACRO version
    # Build the complete path to the xacro file
    xacro_file_name = "arm.urdf.xacro"
    xacro = os.path.join(
        get_package_share_directory('armando_description'), "urdf", xacro_file_name)

    robot_description_xacro = {
        "robot_description": ParameterValue(
            Command(['xacro ', xacro]),  
            value_type=str
        )
    }

    '''# Define and configure the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[params,
                   {"use_sim_time": True},
            ],
    )'''

    # Define and configure the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_xacro,
                   {"use_sim_time": True},  #to synchronize gazebo time and ros2 time
            ],
    )
    
    #Declaration of arguments for Gazebo (gz sim)
    # The gz_args parameter defines how Gazebo should be started
    declared_arguments = [
        DeclareLaunchArgument('gz_args',
            default_value='-r -v 1 empty.sdf',
            description='Arguments for gz_sim',
        )
    ]
    
    # This includes the standard launch file from the ros_gz_sim package
    # It starts the Gazebo simulator using the parameters defined above
    gz_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch',                    
                                      'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args'),
                              'publish_clock': 'true'}.items()
    )

    # Set the initial spawn position of the robot in Gazebo
    position = [0.0, 0.0, 0.6]

    # This node calls the `create` executable from ros_gz_sim
    # It reads the robot_description parameter and spawns the robot in the world
    spawn_rob = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',  # Topic containing the robot's URDF description
            '-name', 'armando',             # Name assigned to the spawned entity in Gazebo
            '-allow_renaming', 'true',      # Avoid name conflicts if another model has same name
            '-x', str(position[0]),         # Spawn position X
            '-y', str(position[1]),         # Spawn position Y
            '-z', str(position[2]),         # Spawn position Z
        ],
    )
 
    # Clock bridge to keep simulation time and ROS2 time synchronized
    clock_bridge = Node(
     package="ros_ign_bridge",
     executable="parameter_bridge",
     arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
     output="screen",
 )
    
    # Camera bridge
    camera_bridge = Node(
        package='ros_ign_bridge', 
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image', 
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo', 
            '--ros-args', 
            '-r', '/camera:=/videocamera', 
        ],
        output='screen'
    )
    
    
    # Combine all declared actions and nodes into the final launch description
    return LaunchDescription(
        declared_arguments
        + [ gz_ign,
            robot_state_publisher_node,
            spawn_rob,
            camera_bridge,
            clock_bridge
        ]
    )
