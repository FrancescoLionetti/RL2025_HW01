import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #URDF version
    '''urdf_file_name = 'arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('armando_description'), 'urdf',
        urdf_file_name
    )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    params = {"robot_description": robot_desc}'''
    
    #XACRO version
    # Build the complete path to the xacro file
    xacro_file_name = "arm.urdf.xacro"
    xacro = os.path.join(
        get_package_share_directory('armando_description'), "urdf", xacro_file_name)
    
    # Use xacro to process the file and create the robot description parameter
    params = {
        "robot_description": ParameterValue(
            Command(['xacro ', xacro]),  
            value_type=str
        )
    }

    #To load a configuration for rzviz visualization
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("rviz_config_file", 
            default_value=PathJoinSubstitution(
                [FindPackageShare("armando_description"), "config", "armando.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz."
        )
    )

    # Robot State Publisher Node 
    # Reads the robot_description (URDF) and /joint_states messages
    # Computes and publishes all the TF transforms between robot links
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
    )
    
    #Joint State Publisher Node
    # Publishes /joint_states messages (angles of all robot joints)
    # GUI version: opens a window with sliders for each joint
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )


    # RViz2 Visualization Node
    # Launches RViz2 to visualize the robot
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],  #To set the configuration contained in config file
    )

    # Return the Launch Description
    # All nodes are added to the launch description and executed together
    return LaunchDescription(declared_arguments +[
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
