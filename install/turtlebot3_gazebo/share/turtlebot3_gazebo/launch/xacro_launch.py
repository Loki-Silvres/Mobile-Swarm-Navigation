import argparse
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
from launch_ros.actions import Node

def run_launch_file_with_args(num_bots, x_poses,y_poses):
    # Initialize the LaunchService
    launch_service = LaunchService()
    world = os.path.join(
        get_package_share_directory('my_world'),
        'worlds',
        'warehouse.world'
    )
    
    rviz_config_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','config.rviz')

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

    for i in range(num_bots):
        ent_name = entity_name + str(i)
        ns_name = ns + str(i)
        
        x_pose_val = x_poses[i]
        y_pose_val = y_poses[i]  
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
                'namespace': ns_name,
                'entity_name': ent_name,
                # 'topic': '/' + ns_name + '/robot_description'
            }.items()
        )

        # Include robot state publisher and spawn commands
        launch_service.include_launch_description(robot_state_publisher_cmd)
        launch_service.include_launch_description(spawn_turtlebot_cmd)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path,'use_sime_time','true'],
        output='screen'
    )
    launch_service.include_launch_description(rviz_node)



    launch_service.run()

def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--num_bots', type=int, default=3, help="Number of robots to spawn.")
    parser.add_argument('--x_pose', type=str, nargs='+', default=[0, 0, 0], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--y_pose', type=str, nargs='+', default=[0, 1, 3], 
                        help="List of x positions for the robots (space-separated).")
    
    args = parser.parse_args()
    
    # Convert x_pose argument into a list of floats
    x_poses = [float(x) for x in args.x_pose]
    y_poses = [float(x) for x in args.y_pose]
    
    return args.num_bots, x_poses,y_poses

if __name__ == '__main__':

    
    num_bots, x_poses, y_poses = parse_args()
    run_launch_file_with_args(num_bots, x_poses,y_poses)
