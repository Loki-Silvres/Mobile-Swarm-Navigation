import argparse
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
from launch_ros.actions import Node
import yaml
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
        launch_arguments={'world': world,'use_sim_time':'true'}.items()
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
        arguments=['-d', rviz_config_path,'use_sime_time','true'],
        output='screen'
    )


    launch_service.include_launch_description(rviz_node)
    map_params_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','multirobot_mapper_params.yaml')
    with open(map_params_path, "r") as file:
        map_data = yaml.safe_load(file)
    for i in range(num_bots):
        ent_name = entity_name + str(i)
        ns_name = ns + str(i)
        odom_frame = ns_name + '/' + 'odom'
        base_frame = ns_name + '/' + 'base_footprint'
        scan_topic = '/' + ns_name + '/' + ns_name + '/scan'
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
        
        with open(yaml_file, "w") as file:
            yaml.dump(data, file, default_flow_style=False)
            
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
            # namespace=ns_name,
            output='screen',
            arguments=['--ros-args','--params-file',yaml_file],
            remappings=[('/map', map_topic)],
        )
       
        launch_service.include_launch_description(robot_state_publisher_cmd)
        launch_service.include_launch_description(spawn_turtlebot_cmd)
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
