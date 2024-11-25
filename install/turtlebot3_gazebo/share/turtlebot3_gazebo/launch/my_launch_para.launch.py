#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def my_func(context,*args,**kwargs):
    
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = int(LaunchConfiguration('x_pose').perform(context))
    y_pose = int(LaunchConfiguration('y_pose').perform(context))
    num_bots = int(LaunchConfiguration('num_of_bots').perform(context))
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )
    print(num_bots)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )


    # Initialize variables for robot names and namespaces
    entity_name = 'waffle_'
    ns = 'bot_'
    robot_publisher_nodes = []
    robot_spawn_nodes = []

    x_pose_cmd = []
    y_pose_cmd = []
    
    # Loop to spawn robots based on launch argument
    for i in range(num_bots):
        ent_name = entity_name + str(i)
        ns_name = ns + str(i)
        x_pose_val = x_pose+i  
        y_pose_val = y_pose+i  
        x_pose_arg_name = f'x_pose_{i}'
        y_pose_arg_name = f'y_pose_{i}'

        x_pose_arg = DeclareLaunchArgument(x_pose_arg_name, default_value=str(x_pose_val))
        y_pose_arg = DeclareLaunchArgument(y_pose_arg_name, default_value=str(y_pose_val))


        # Retrieve the launch configuration
        x_pose_str = LaunchConfiguration(x_pose_arg_name)
        y_pose_str = LaunchConfiguration(y_pose_arg_name)

        x_pose_cmd.append(x_pose_arg)
        y_pose_cmd.append(y_pose_arg)
        

        # Include robot state publisher launch file
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'my_robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time, 'namespace': ns_name}.items()
        )

        # Include spawn turtlebot launch file
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'my_spawner.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_str,
                'y_pose': y_pose_str,
                'namespace': ns_name,
                'entity_name': ent_name
            }.items()
        )

        # Add nodes to their respective lists
        robot_publisher_nodes.append(robot_state_publisher_cmd)
        robot_spawn_nodes.append(spawn_turtlebot_cmd)
    return ([gzserver_cmd,gzclient_cmd]+x_pose_cmd+y_pose_cmd+robot_publisher_nodes+robot_spawn_nodes)

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('num_of_bots', default_value='1', description='Number of robots to spawn'))
    ld.add_action(DeclareLaunchArgument('x_pose',default_value='1'))
    ld.add_action(DeclareLaunchArgument('y_pose',default_value='1'))

    ld.add_action(OpaqueFunction(function=my_func))

    return ld