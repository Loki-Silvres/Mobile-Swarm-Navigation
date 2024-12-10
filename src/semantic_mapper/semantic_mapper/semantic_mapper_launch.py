#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchService
import argparse

def run_launch_file_with_args(bot_name):

    semantic_node = Node(
        package='semantic_mapper',
        executable='semantic_mapper',
        name='mapper',
        output='screen',
        parameters=[
            {'bot_name': bot_name},
            {'use_sim_time':True}
            
        ]
    )

    changer_node = Node(
        package='frame_changer',
        executable='frame_changer',
        name='changer_node',
        output = 'screen',
        parameters=[
            {'use_sim_time':True},
            {'ns_name':bot_name}
        ]
    )
    ls = LaunchService()
    
    ls.include_launch_description(changer_node)
    ls.include_launch_description(semantic_node)


    ls.run()
    
        
def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--bot_name', type=str, default='bot_0', help="Name pf bot to be added")
    args = parser.parse_args()
    
    return args.bot_name

if __name__ == '__main__':

    
    bot_name = parse_args()
    run_launch_file_with_args(bot_name)