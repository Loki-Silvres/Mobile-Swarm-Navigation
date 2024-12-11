from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchService
import argparse
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
def run_launch_file_with_args(num_bots):


    interpreter = Node(
        package='interpreter',
        executable='interpreter',
        name='interpreter',
        output='screen',
        
    )

    scheduler = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_share_directory('scheduler'),'launch','scheduler_launch.py'),'--num_bots='+str(num_bots)],
            output='screen',
            )
    ls = LaunchService()
    
    ls.include_launch_description(interpreter)
    ls.include_launch_description(scheduler)
    

    ls.run()
    
        
def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--num_bots', type=int, default=4, help="number_of_bots")
    args = parser.parse_args()
    
    return args.num_bots

if __name__ == '__main__':

    
    num_bots = parse_args()
    run_launch_file_with_args(num_bots)