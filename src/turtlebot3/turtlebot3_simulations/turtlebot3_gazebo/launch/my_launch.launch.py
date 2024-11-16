#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    DeclareLaunchArgument('num_of_bots', default_value='1', description='Number of robots to spawn')
    DeclareLaunchArgument('x_pose',default_value='1.0')
    DeclareLaunchArgument('y_pose',default_value='1.0')
    
    num_bots = LaunchConfiguration('num_of_bots')
    ld.add_action(num_bots)

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = 1.0
    y_pose = 1.0
    x_pose_value = LaunchConfiguration("x_pose").perform(None)
    y_pose_value = LaunchConfiguration("y_pose").perform(None)
    
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

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

    # Log the number of robots to be spawned
    ld.add_action(LogInfo(msg=f"Spawning {num_bots.perform(None)} robots"))

    # Initialize variables for robot names and namespaces
    entity_name = 'waffle_'
    ns = 'bot_'
    robot_publisher_nodes = []
    robot_spawn_nodes = []

    # Loop to spawn robots based on launch argument
    for i in range(int(num_bots.perform(None))):
        ent_name = entity_name + str(i)
        ns_name = ns + str(i)
        x_pose = f'{float(x_pose_value) + i}'  
        y_pose = f'{float(y_pose_value) + i}'  
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
                'x_pose': x_pose,
                'y_pose': y_pose,
                'namespace': ns_name,
                'entity_name': ent_name
            }.items()
        )

        # Add nodes to their respective lists
        robot_publisher_nodes.append(robot_state_publisher_cmd)
        robot_spawn_nodes.append(spawn_turtlebot_cmd)

    # Add spawn and robot publisher nodes to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    for cmd in robot_spawn_nodes + robot_publisher_nodes:
        ld.add_action(cmd)


    return ld