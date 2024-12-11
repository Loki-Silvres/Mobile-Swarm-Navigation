import argparse
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
from launch_ros.actions import Node
import yaml
def run_launch_file_with_args(num_bots):
    # Initialize the LaunchService
    launch_service = LaunchService()
    ns = 'bot_'
    yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','params_costmap.yaml')
    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)
    
    for i in range(num_bots):
        ns_name = ns + str(i)
        data['explore_node']['ros__parameters']['robot_base_frame'] = ns_name + '/base_link'
        data['explore_node']['ros__parameters']['costmap_topic'] = '/' + ns_name + '/global_costmap/costmap'
        data['explore_node']['ros__parameters']['costmap_updates_topic'] = '/' + ns_name + '/global_costmap/costmap_updates'
        # data1 = dict()
        # data1[ns_name] = data
        yaml_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'config','params_costmap' + str(i) + '.yaml')


        remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

        node = Node(
            package="explore_lite",
            name="explore_node",
            namespace='',
            executable="explore",
            parameters=[yaml_file, {"use_sim_time": True},{'ns_name':ns_name}],
            output="screen",
            remappings=remappings,
        )

        with open(yaml_file, "w") as file:
            yaml.dump(data, file, default_flow_style=False)
        
        launch_service.include_launch_description(node)        
    launch_service.run()

def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--num_bots', type=int, default=3, help="Number of robots to spawn.")
    
    args = parser.parse_args()
    
    
    return args.num_bots

if __name__ == '__main__':

    
    num_bots = parse_args()
    run_launch_file_with_args(num_bots)
