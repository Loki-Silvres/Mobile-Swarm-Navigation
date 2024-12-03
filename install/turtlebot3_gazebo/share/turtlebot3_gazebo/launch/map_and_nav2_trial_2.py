import argparse
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService,LaunchDescription
from launch.actions import IncludeLaunchDescription,RegisterEventHandler,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
from launch_ros.actions import Node
import yaml

from launch.event_handlers import OnProcessExit
def run_launch_file_with_args(num_bots, x_poses,y_poses,z_poses):
    # Initialize the LaunchService
    launch_service = LaunchService()
    world = os.path.join(
        get_package_share_directory('my_world'),
        'worlds',
        'warehouse_1.world'
    )
    #world = os.path.join(
    #    get_package_share_directory('turtlebot3_gazebo'),
    #    'worlds',
    #    'turtlebot3_world.world'
    #)
    
    yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','mapper_params_online_async.yaml')

    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)
    
    rviz_config_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','config_map_merger.rviz')

    # Path to launch files
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={
            # 'world': world,
            'use_sim_time':'true'}.items()
    )



    entity_name = 'waffle_'
    ns = 'bot_'
    launch_service.include_launch_description(spawn_gazebo)

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path,'use_sime_time','true'],
    #     output='screen'
    # )


    # launch_service.include_launch_description(rviz_node)
    map_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','multirobot_mapper_params.yaml')
    with open(map_params_path, "r") as file:
        map_data = yaml.safe_load(file)



    for i in range(num_bots):
         ns_name = ns + str(i)
         frame_changer=Node(
            package='frame_changer',
            executable='frame_changer',
            name='frame_changer_node',
                # namespace=ns_name,
            output='screen',
            parameters=[
                {'ns_name': ns_name},     # Pass parameters here
                {'use_sim_time': True} 
            ]
         )

         launch_service.include_launch_description(frame_changer)


    
    for i in range(num_bots):
        ent_name = entity_name + str(i)
        ns_name = ns + str(i)
        odom_frame = ns_name + '/' + 'odom'
        base_frame = ns_name + '/' + 'base_footprint'
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




        

        yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','mapper_params_online_async_'+str(i)+'.yaml')
        
        

        data['slam_toolbox']['ros__parameters']['odom_frame']=odom_frame
        data['slam_toolbox']['ros__parameters']['base_frame']=base_frame
        data['slam_toolbox']['ros__parameters']['scan_topic']=scan_topic
        
        x_pose_val = x_poses[i]
        y_pose_val = y_poses[i]  
        z_pose_val = z_poses[i]  
        
        b_data = {ns_name:data}
        with open(yaml_file, "w") as file:
            yaml.dump(b_data, file, default_flow_style=False)
                
        

        my_slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns_name,
            output='screen',
            arguments=['--ros-args','--params-file',yaml_file],
            remappings=[('/map', map_topic),('/scan','/'+ns_name+'/scan')],
        )
        
        launch_service.include_launch_description(my_slam_toolbox)

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

    last_action = None
    for i in range(num_bots):
        ns_name = ns + str(i)


        nav2_process = ExecuteProcess(
            cmd=[
                'ros2', 'launch', 
                os.path.join(get_package_share_directory('turtlebot3_multi_robot'),'launch','gazebo_multi_nav2_world.launch.py'),
                f'ns_name:={ns_name}',
                f'x_pose:={x_poses[i]}',
                f'y_pose:={y_poses[i]}',
                f'z_pose:={z_poses[i]}',
                
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
    parser.add_argument('--x_pose', type=str, nargs='+', default=[0.0, 0.0, 0.0, 0.0], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--y_pose', type=str, nargs='+', default=[1.0, 1.5, 2.0, 2.5], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--z_pose', type=str, nargs='+', default=[0.05, 0.05, 0.05, 0.05], 
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
