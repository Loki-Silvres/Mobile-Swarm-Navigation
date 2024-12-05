#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_robot_description(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(get_package_share_directory('turtlebot3_xacro'), 'urdf', 'turtlebot3_manipulation.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"prefix":namespace+'/'})
    robot_desc = doc.toprettyxml()
    my_joint_state_topic = namespace + '/joint_states'
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
            remappings=[('/joint_states', my_joint_state_topic)]
            
        )
    ]

def generate_launch_description():
    # Declare input arguments for x, y, and z coordinates (can be provided during launch)
    x_arg = DeclareLaunchArgument('x', default_value='192.0', description='X coordinate for joint target')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Y coordinate for joint target')
    z_arg = DeclareLaunchArgument('z', default_value='131.0', description='Z coordinate for joint target')
    
    return LaunchDescription([
        
        x_arg,
        y_arg,
        z_arg,
        Node(
            package='turtlebot3_gazebo',  # Replace with the correct package name
            executable='joint_angle_publish.py',  # Ensure this script is in the correct folder and executable
            name='joint_angle_publisher',  # Name of the node
            output='screen',  # Output logs to the terminal
            parameters=[
                # Pass the x, y, z arguments as parameters to the node
                {'x': LaunchConfiguration('x')},
                {'y': LaunchConfiguration('y')},
                {'z': LaunchConfiguration('z')}
            ]
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='bot_0',
            description='Namespace for the robot'
        ),
        OpaqueFunction(function=generate_robot_description)
    ])
