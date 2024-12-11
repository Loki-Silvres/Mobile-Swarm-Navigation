#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.events import Shutdown



def generate_launch_description():
    ld = LaunchDescription()
    # initializations
    package_dir = get_package_share_directory('auto_explore')
    launch_dir = os.path.join(package_dir, 'launch')
    nav_bringup_dir = os.path.join(launch_dir, 'bringup')

    
    
    bot_desc= {'name':'amr','x_pose':2.50,'y_pose':0.4,'z_pose':0.01}

    BOT_MODEL = 'waffle_pi'

    urdf = os.path.join(
        package_dir, 'urdf', 'turtlebot3_' + BOT_MODEL + '.urdf'
    )
    
    
    
    
    # If warehouse world has to be simulated, uncomment the following and use the below bot_desc variable, comment the previous one 
    # world = '/home/loki/auto_explore_ws/src/aws-robomaker-small-warehouse-world-ros2/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world'
    # bot_desc = {'name':'amr','x_pose':-2.75,'y_pose':4.50,'z_pose':0.01}
    

    # launch configurations
    namespace = LaunchConfiguration('namespace')
    slam = LaunchConfiguration('slam')
    enable_rviz =LaunchConfiguration('enable_rviz', default='true')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_namespace = LaunchConfiguration('use_namespace')

    world = LaunchConfiguration('world')
    
    world_gz = LaunchConfiguration('world_gz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    





    #launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='True', description='Whether run a SLAM'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='true',description='Whether use namespace or not'
    )

    declare_use_sim_time = DeclareLaunchArgument(
    name='use_sim_time', default_value='true', description='Use simulator time'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'rviz_config.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('aws_robomaker_hospital_world'), 'worlds', 'hospital.world'),
        description='Full path to world model file to load',
    )

    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value='hospital',
    #     description='Name of the world',
    # )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(package_dir, 'maps', 'map.yaml'),
    )
    
    
    # launch gazebo
    

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time':'true'}.items()
    )

    # launch nodes

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    
    turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
    )

    spawn_bot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-file', os.path.join(package_dir, 'models', 'turtlebot3_' + BOT_MODEL, 'model.sdf'),
        '-entity', bot_desc['name'],
        '-robot_namespace', namespace,
        '-x', str(bot_desc['x_pose']),
        '-y', str(bot_desc['y_pose']),
        '-z', str(bot_desc['z_pose']),
        '-R', '0.0',
        '-P', '0.0',
        '-Y', '1.57',
        '-unpause',
    ],
    output='screen',
    )
    
    rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_bringup_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': use_namespace,
                                  'rviz_config': rviz_config_file, 
                                  'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )


    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_bringup_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    explore_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')),
        
    )

    # map_save_trial
   


    

    # message = f"{{header: {{frame_id: 'map'}}, pose: {{pose: {{position: {{x: {bot_desc['x_pose']}, y: {bot_desc['y_pose']}, z: {bot_desc['z_pose']}}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}}}}}}}"

    # # Command to publish the initial pose only once
    # initial_pose_cmd = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'topic', 'pub',
    #          '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', message, '-1'
    #     ],
    #     output='screen'
    # )
    # delay_initial_pose_cmd = TimerAction(period=0.25, actions=[initial_pose_cmd])


    goal_pose = {
        'x': 0.0,  # Set your desired X-coordinate
        'y': 1.0,  # Set your desired Y-coordinate
        'z': 0.0   # Z is usually 0 for 2D navigation
    }

    orientation = {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'w': 1.0  # Orientation quaternion (facing forward)
    }

    # Create the message to publish
    message = (
        f"{{"
        f"header: {{frame_id: 'map'}}, "
        f"pose: {{"
        f"position: {{x: {goal_pose['x']}, y: {goal_pose['y']}, z: {goal_pose['z']}}}, "
        f"orientation: {{x: {orientation['x']}, y: {orientation['y']}, z: {orientation['z']}, w: {orientation['w']}}}"
        f"}}"
        f"}}"
    )

    # Command to publish the navigation goal pose
    goal_pose_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '/goal_pose', 'geometry_msgs/PoseStamped', message, '-1'  # Publish once
        ],
        output='screen'
    )

    delay_action = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[goal_pose_cmd]
    )
    
    
    
    
    
    
    
    # populate launch description

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_namespace_cmd)


    
    ld.add_action(gazebo_launch)
    ld.add_action(turtlebot_state_publisher)
    ld.add_action(spawn_bot)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(explore_cmd)

    
    
    # trials......
    # ld.add_action(initial_pose_cmd)
    # ld.add_action(on_shutdown)
    # ld.add_action(goal_pose_cmd)
    # ld.add_action(goal_pose_event_handler)
    # ld.add_action(delay_action)
    # ld.add_action(delay_initial_pose_cmd)
    
    
    
    
    return ld