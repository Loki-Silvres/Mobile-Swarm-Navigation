#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging
import xacro
def generate_robot_description(context,*args,**kwargs):
    ld = []

    # Names and poses of the robots
    # robots = [
    #     {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
    #     {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
    #     {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
    #     {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
    #     # ...
    #     # ...
    #     ]

    TURTLEBOT3_MODEL = 'waffle'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    enable_drive = LaunchConfiguration('enable_drive', default='false')

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    bot_name = LaunchConfiguration('bot_name').perform(context)
    robots = [
        {'name':bot_name,'x_pose':str(x_pose),'y_pose':str(y_pose),'z_pose':str(z_pose)},
    ]
    
    
    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    package_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # urdf = os.path.join(
    #     turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    # )

    xacro_file = os.path.join(get_package_share_directory('turtlebot3_xacro'), 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"prefix":bot_name+'/'})
    urdf = doc.toprettyxml()
    
    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_multi_robot'),
    #     'worlds', 'multi_robot_world.world')

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items(),
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
    #     ),
    # )

    params_file = LaunchConfiguration('nav_params_file')


    # ld.append(gzserver_cmd)
    # ld.append(gzclient_cmd)
 
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    # map_server=Node(package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
    #                  },],
    #     remappings=remappings)

    # map_server_lifecyle=Node(package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_map_server',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time},
    #                     {'autostart': True},
    #                     {'node_names': ['map_server']}])


    # ld.append(map_server)
    # ld.append(map_server_lifecyle)

    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        # namespace = [ '/' + robot['name'] ]

        namespace = [robot['name'] ]

        # Create state publisher node for that instance
        # turtlebot_state_publisher = Node(
        #     package='robot_state_publisher',
        #     namespace=namespace,
        #     executable='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time,
        #                     'publish_frequency': 10.0}],
        #     remappings=remappings,
        #     arguments=[urdf],
        # )

        my_joint_state_topic = bot_name + '/joint_states'
        turtlebot_state_publisher=Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    namespace=bot_name,
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': urdf,
                        'publish_frequency': 100.0,
                    }],
                    remappings=[('/joint_states', my_joint_state_topic)]
        )       
        # Create spawn call
        # spawn_turtlebot3_burger = Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=[
        #         '-file', os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
        #         '-entity', robot['name'],
        #         '-robot_namespace', namespace,
        #         '-x', robot['x_pose'], '-y', robot['y_pose'],
        #         '-z', robot['z_pose'], '-Y', '0.0',
        #         '-unpause',
        #     ],
        #     output='screen',
        # )

        my_robot_desc = bot_name + '/robot_description'
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'bot_0',
                '-topic', my_robot_desc,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-robot_namespace', bot_name
            ],
            output='screen',
        )


        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )

        if last_action is None:
            # Call append directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.append(turtlebot_state_publisher)
            ld.append(spawn_turtlebot3_burger)
            ld.append(bringup_cmd)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.append for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                            turtlebot_state_publisher,
                            bringup_cmd],
                )
            )

            ld.append(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        # namespace = [ '/' + robot['name'] ]

        namespace = [ robot['name'] ]

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'
        
        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        # Use RegisterEventHandler to ensure next robot rviz launch happens 
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        last_action = initial_pose_cmd

        ld.append(post_spawn_event)
    ######################

    return ld



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='use_sim_time', default_value='true', description='Use simulator time'
        ),

        DeclareLaunchArgument(
            name='enable_drive', default_value='false', description='Enable robot drive node'
        ),
        DeclareLaunchArgument(
            name='enable_rviz', default_value='true', description='Enable rviz launch'
        ),

        DeclareLaunchArgument('x_pose',default_value='0.0'),
        DeclareLaunchArgument('y_pose',default_value='0.0'),
        DeclareLaunchArgument('z_pose',default_value='0.0'),
        DeclareLaunchArgument('bot_name',default_value='bot_0'),
        DeclareLaunchArgument(
                'rviz_config_file',
                default_value=os.path.join(
                    get_package_share_directory('turtlebot3_multi_robot'), 'rviz', 'multi_nav2_default_view.rviz'),
                description='Full path to the RVIZ config file to use'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(get_package_share_directory('turtlebot3_multi_robot'), 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        


        OpaqueFunction(function=generate_robot_description)
    ])

