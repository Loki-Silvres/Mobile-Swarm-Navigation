import argparse
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
from launch_ros.actions import Node
import yaml
from launch.event_handlers import OnProcessExit
from launch_ros.actions import PushRosNamespace,LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
def run_launch_file_with_args(num_bots, x_poses,y_poses,z_poses):
    # Initialize the LaunchService
    launch_service = LaunchService()
    # world = os.path.join(
    #     get_package_share_directory('my_world'),
    #     'worlds',
    #     'warehouse_1.world'
    # )
    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'worlds',
    #     'turtlebot3_world.world'
    # )
    world = os.path.join(
        get_package_share_directory('aws_robomaker_hospital_world'),
        'worlds',
        'hospital.world'
    )
   
    yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','mapper_params_online_async_mine.yaml')

    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)
    
    rviz_config_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','config_map_merger.rviz')

    # Path to launch files
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={'world': world,
                          'use_sim_time':'true'
                          }.items()
    )


    # world_launch_file = os.path.join(get_package_share_directory('my_world'),'launch','bf_world.launch.py')
    # my_world_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(world_launch_file)
    # )
    # launch_service.include_launch_description(my_world_cmd)

    # Parameters for spawning multiple robots
    entity_name = 'waffle_'
    ns = 'bot_'
    launch_service.include_launch_description(spawn_gazebo)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path,
                   'use_sime_time','true'
                   ],
        output='screen'
    )


    launch_service.include_launch_description(rviz_node)
    map_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','multirobot_mapper_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','nav2_params.yaml')
    params_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','nav2_params_1.yaml')
    with open(map_params_path, "r") as file:
        map_data = yaml.safe_load(file)
    with open(nav2_params_path, "r") as file:
        nav2_data = file.read()
    last_action = None


    local_costmap_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','local_costmap_params.yaml')
    with open(local_costmap_params_path,'r') as file1:
        local_costmap_data = file1.read()


    planner_server_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','planner_server_params.yaml')
    with open(planner_server_params_path,'r') as file2:
        planner_server_data = file2.read()



    for i in range(num_bots):
        ent_name = entity_name + str(i)
        ns_name = ns + str(i)
        odom_frame = ns_name + '/' + 'odom'
        base_frame = ns_name + '/' + 'base_footprint'
        # scan_topic = '/' + ns_name + '/' + ns_name + '/scan'
        scan_topic = '/' + ns_name + '/scan'
        
        map_topic = '/' + ns_name + '/map'
        x_param = '/' + ns_name + '/map_merge/init_pose_x'
        y_param = '/' + ns_name + '/map_merge/init_pose_y'
        z_param = '/' + ns_name + '/map_merge/init_pose_z'
        yaw_param = '/' + ns_name + '/map_merge/init_pose_yaw'

        
        map_data['map_merge']['ros__parameters'][x_param] = x_poses[i]
        map_data['map_merge']['ros__parameters'][y_param] = y_poses[i]
        map_data['map_merge']['ros__parameters'][z_param] = z_poses[i]
        map_data['map_merge']['ros__parameters'][yaw_param] = 0.0


        nav2_data_write = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','nav2_params_'+str(i)+'.yaml')
        nav2_data = nav2_data.replace('bot_0',ns_name)
        with open(nav2_data_write, 'w') as file:
            file.write(nav2_data)
        
        local_costmap_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','local_costmap_params_'+str(i)+'.yaml')
        local_costmap_data_write = local_costmap_data.replace('bot_0',ns_name)
        with open(local_costmap_params_path,'w') as file1:
            file1.write(local_costmap_data_write)

        planner_server_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','planner_server_params_'+str(i)+'.yaml')
        planner_server_data = planner_server_data.replace('bot_0',ns_name)
        with open(planner_server_params_path,'w') as file2:
            file2.write(planner_server_data)

        
        

        yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','mapper_params_online_async_'+str(i)+'.yaml')
        
        

        data['slam_toolbox']['ros__parameters']['odom_frame']=odom_frame
        data['slam_toolbox']['ros__parameters']['base_frame']=base_frame
        data['slam_toolbox']['ros__parameters']['scan_topic']=scan_topic
        b_data = {ns_name:data}
        with open(yaml_file, "w") as file:
            yaml.dump(b_data, file, default_flow_style=False)
            
        x_pose_val = x_poses[i]
        y_pose_val = y_poses[i]  
        z_pose_val = z_poses[i]  
        
        # Robot state publisher
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'xacro_robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true', 'namespace': ns_name}.items()
        )

        # Spawn robot with its pose and namespace
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'xacro_spawn.launch.py')
            ),
            launch_arguments={
                'x_pose': f"{x_pose_val}",
                'y_pose': f"{y_pose_val}",
                'z_pose': f"{z_pose_val}",
                'namespace': ns_name,
                'entity_name': ent_name,
                # 'topic': '/' + ns_name + '/robot_description'
            }.items()
        )

        my_slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns_name,
            output='screen',
            arguments=['--ros-args','--params-file',yaml_file],
            remappings=[('/map', map_topic),('/scan','/'+ns_name+'/scan')],
        )
        
        # local_costmap_node =Node(
        #     package="nav2_costmap_2d",
        #     executable="nav2_costmap_2d",
        #     name="local_costmap",
        #     output="screen",
        #     parameters=[local_costmap_params_path],
        #     remappings=[
        #     ('/costmap/costmap', '/'+ns_name+'/costmap/costmap'),
        #     ('/map','/'+ns_name+'/map'),
        #     ]
        # )

        # lifecycle_manager_node=Node(
        # package='nav2_lifecycle_manager',
        # executable='lifecycle_manager',
        # name='lifecycle_manager_navigation',
        # output='screen',
        # parameters=[{'use_sim_time': True},
        #             {'autostart': True},
        #             {'node_names': ['/costmap/costmap']}])


        # planner_server_node = Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[planner_server_params_path],
        #     remappings=[
        #         ('/planner_server/input_topic', '/custom_costmap'),
        #         ('/planner_server/output_topic', '/custom_cmd_vel'),
        #     ],
        # )
        # lifecycle_manager_node_ps=Node(
        # package='nav2_lifecycle_manager',
        # executable='lifecycle_manager',
        # name='lifecycle_manager_navigation',
        # output='screen',
        # parameters=[{'use_sim_time': True},
        #             {'autostart': True},
        #             {'node_names': ['/planner_server']}])

        
        # remappings=[("/goal_pose","/"+ns_name+"/goal_pose")]
        # grp_node = GroupAction([
        #     # PushRosNamespace(namespace='/'+ns_name),
            
        #     IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'),'launch','navigation_launch.py')]),
        #     launch_arguments={'namespace': '/' + ns_name,'use_sim_time':'true',
        #                       'params_file':nav2_params_path
        #                       }.items(),
        #     )
        # ])
        # my_nav2_bringup = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'),'launch','navigation_launch.py')]),
        #     launch_arguments={'namespace': ns_name,'use_sim_time':'true','params_file':nav2_data_write}.items(),
        # )
        # launch_service.include_launch_description(my_nav2_bringup)
        
        
        # bringup_cmd = IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             os.path.join(get_package_share_directory('turtlebot3_multi_robot'),'launch','nav2_bringup' ,'bringup_launch.py')),
        #             launch_arguments={  
        #                             'slam': 'False',
        #                             'namespace': '/'+ ns_name,
        #                             'use_namespace': 'True',
        #                             'map': '',
        #                             'map_server': 'False',
        #                             'params_file': params_file,
        #                             'default_bt_xml_filename': os.path.join(
        #                                 get_package_share_directory('nav2_bt_navigator'),
        #                                 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        #                             'autostart': 'true',
        #                             'use_sim_time': 'true', 'log_level': 'warn'}.items()
        # )    
        

        launch_service.include_launch_description(robot_state_publisher_cmd)
        launch_service.include_launch_description(spawn_turtlebot_cmd)
        launch_service.include_launch_description(my_slam_toolbox)
        # launch_service.include_launch_description(local_costmap_node)
        # launch_service.include_launch_description(lifecycle_manager_node)
        # launch_service.include_launch_description(planner_server_node)
        # launch_service.include_launch_description(lifecycle_manager_node_ps)

        # launch_service.include_launch_description(LaunchDescription([grp_node]))
        
        # launch_service.include_launch_description(bringup_cmd)                    

    with open(map_params_path, "w") as file:
        yaml.dump(map_data, file, default_flow_style=False)

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    mapper_node = Node(
        package="map_merge",
        name="map_merge",
        namespace='/',
        executable="map_merge",
        parameters=[
            map_params_path,
            'use_sim_time', 'true',
        ],
        output="screen",
        remappings=remappings,
    )
    launch_service.include_launch_description(mapper_node)

    # entity_name = 'waffle_'
    # ns = 'bot_'
    # last_action = None
    # for i in range(num_bots):
    #     ent_name = entity_name + str(i)
    #     ns_name = ns + str(i)
    
    #     namespace = [ '/' + ns_name ]
    #     launch_exec = ExecuteProcess(
    #         cmd=['ros2','launch','turtlebot3_multi_robot','bringup_launch.py','slam:=False','namespace:='+ '/'+ ns_name,'use_namespace:='+'True',
    #              'map:='+' ','map_server:='+'False','params_file:='+ params_file,'default_bt_xml_filename:='+ os.path.join(
    #                                     get_package_share_directory('nav2_bt_navigator'),
    #                                     'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
    #                                 'autostart:='+ 'true',
    #                                 'use_sim_time:='+'true', 'log_level:='+ 'warn']
    #     )

    #     launch_cmd = LaunchDescription([launch_exec])
    #     if last_action is None:
    #         # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
    #         launch_service.include_launch_description(launch_cmd)


    #     else:
    #         spawn_turtlebot3_event = RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=last_action,
    #                 on_exit=[launch_cmd],
    #             )
    #         )

    #         launch_service.include_launch_description(LaunchDescription([spawn_turtlebot3_event]))

    #     last_action = launch_exec


    # entity_name = 'waffle_'
    # ns = 'bot_'
    # for i in range(num_bots):
    #     ent_name = entity_name + str(i)
    #     ns_name = ns + str(i)
    
    #     namespace = [ '/' + ns_name ]

    #     # Create a initial pose topic publish call
    #     message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
    #         str(x_poses[i]) + ', y: ' + str(y_poses[i]) + \
    #         ', z: ' + \
    #         str(z_poses[i]) + '}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

    #     initial_pose_cmd = ExecuteProcess(
    #         cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
    #             'geometry_msgs/PoseWithCovarianceStamped', message],
    #         output='screen'
    #     )
    #     drive_turtlebot3_burger = Node(
    #         package='turtlebot3_gazebo', executable='turtlebot3_drive',
    #         namespace=namespace, output='screen'
    #     )


    #     # launch_service.include_launch_description(initial_pose_cmd)
    #     launch_service.include_launch_description(drive_turtlebot3_burger)
    # global_costmap_node =Node(
    #         package="nav2_costmap_2d",
    #         executable="nav2_costmap_2d",
    #         name="global_costmap",
    #         output="screen",
    #         namespace='my_ns',
    #         parameters=[os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','global_costmap_params.yaml')]
    #         )
    # lifecycle_manager=Node(
    # package='nav2_lifecycle_manager',
    # executable='lifecycle_manager',
    # name='lifecycle_manager_navigation',
    # output='screen',
    # parameters=[{'use_sim_time': True},
    #             {'autostart': True},
    #             {'node_names': ['/costmap/costmap']}])
    



    # launch_service.include_launch_description(LaunchDescription([global_costmap_node]))
    # launch_service.include_launch_description(lifecycle_manager)
    launch_service.run()

def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--num_bots', type=int, default=3, help="Number of robots to spawn.")
    parser.add_argument('--x_pose', type=str, nargs='+', default=[0.0, 0.0, 0.0], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--y_pose', type=str, nargs='+', default=[1.0, 1.5, 2.0], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--z_pose', type=str, nargs='+', default=[0.05, 0.05, 0.05], 
                        help="List of x positions for the robots (space-separated).")
    
    args = parser.parse_args()
    
    # Convert x_pose argument into a list of floats
    x_poses = [float(x) for x in args.x_pose]
    y_poses = [float(x) for x in args.y_pose]
    z_poses = [float(x) for x in args.z_pose]
    
    return args.num_bots, x_poses,y_poses,z_poses

if __name__ == '__main__':

    
    num_bots, x_poses, y_poses,z_poses = parse_args()
    run_launch_file_with_args(num_bots, x_poses,y_poses,z_poses)
