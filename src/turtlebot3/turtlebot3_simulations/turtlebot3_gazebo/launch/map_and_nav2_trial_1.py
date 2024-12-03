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
import subprocess
import threading
import lifecycle_msgs.msg
def run_launch_file_with_args(num_bots, x_poses,y_poses,z_poses):
    # Initialize the LaunchService
    launch_service = LaunchService()
    world = os.path.join(
        get_package_share_directory('my_world'),
        'worlds',
        'warehouse_1.world'
    )
    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'worlds',
    #     'turtlebot3_world.world'
    # )
    
    yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','mapper_params_online_async_mine.yaml')

    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)
    
    rviz_config_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','config_map_merger.rviz')

    # Path to launch files
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={
                        #   'world': world,
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


    nav2_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'navigation_launch.py'
    )

    nav2_params_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'config',
        'nav2_params.yaml'
    )

    with open(nav2_params_file,'r') as file3:
        nav2_data_mine = file3.read()


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
        

        nav2_params_file = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'config',
            'nav2_params_'+str(i)+'.yaml'
        )

        nav2_data_write_mine = nav2_data_mine.replace('bot_0',ns_name)
        with open(nav2_params_file,'w') as file3:
            file3.write(nav2_data_write_mine)        


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
        
        # nav2_process = ExecuteProcess(
        #     cmd=[
        #         'ros2', 'launch', 
        #         nav2_launch_file,
        #         f'params_file:={nav2_params_file}',
        #         'use_sim_time:=true',
        #         f'ns_name:={ns_name}'
        #     ],
        #     output='screen',
        # )
    
        launch_service.include_launch_description(robot_state_publisher_cmd)
        launch_service.include_launch_description(spawn_turtlebot_cmd)
        launch_service.include_launch_description(my_slam_toolbox)
        # execute_command_in_thread(
        #         [
        #         'ros2', 'launch', 
        #         nav2_launch_file,
        #         f'params_file:={nav2_params_file}',
        #         'use_sim_time:=true',
        #         f'ns_name:={ns_name}'
        #     ]
        # )
        # launch_service.include_launch_description(nav2_process)

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

    # nav2_stack = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('turtlebot3_gazebo'),'launch','navigation_launch.py')),
    #     launch_arguments={'params_file':os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','nav2_params.yaml'),'use_sim_true':'true',
    #                       'ns_name':ns_name}.items()
    # )
    # launch_service.include_launch_description(nav2_stack)


    # Create the ExecuteProcess action to run the launch file
    last_action = None
    for i in range(num_bots):
        ns_name = ns + str(i)

        nav2_params_file = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'config',
            'nav2_params_'+str(i)+'.yaml'
        )

        nav2_process = ExecuteProcess(
            cmd=[
                'ros2', 'launch', 
                nav2_launch_file,
                f'params_file:={nav2_params_file}',
                'use_sim_time:=true',
                f'ns_name:={ns_name}'
            ],
            output='screen',
        )

        if last_action is None:
            launch_service.include_launch_description(nav2_process)
        else:
            nav2_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[nav2_process]
                )
            )
            launch_service.include_launch_description(LaunchDescription([nav2_event]))
        last_action = nav2_process    


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


def execute_command_in_thread(command):
    def target():
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        stdout, stderr = process.communicate()
        print(f"Command '{' '.join(command)}' finished.")
        if stdout:
            print(f"Output:\n{stdout}")
        if stderr:
            print(f"Errors:\n{stderr}")

    # Create a new thread to run the command
    thread = threading.Thread(target=target)
    thread.daemon = True  # Optional: allows program to exit even if thread is running
    thread.start()

if __name__ == '__main__':

    
    num_bots, x_poses, y_poses,z_poses = parse_args()
    run_launch_file_with_args(num_bots, x_poses,y_poses,z_poses)
