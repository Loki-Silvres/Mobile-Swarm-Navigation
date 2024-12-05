from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchService
import argparse

def run_launch_file_with_args(num_bots):
    primary_scheduler = Node(
        package='scheduler',
        executable='scheduler_3',
        name='primary_scheduler',
        output = 'screen',
        parameters=[
            {'use_sim_time':True},
            {'num_bots':num_bots}
        ]

        
    )

    secondary_scheduler = Node(
        package='scheduler',
        executable='sub_scheduler',
        name='second_scheduler',
        output = 'screen',
        parameters=[
            {'use_sim_time':True},
            {'num_bots':num_bots}
        ]
    )
    ls = LaunchService()
    ls.include_launch_description(primary_scheduler)
    ls.include_launch_description(secondary_scheduler)
    ls.run()
    
        
def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--num_bots', type=int, default=3, help="Number of robots to spawn.")
    args = parser.parse_args()
    
    return args.num_bots

if __name__ == '__main__':

    
    num_bots = parse_args()
    run_launch_file_with_args(num_bots)
