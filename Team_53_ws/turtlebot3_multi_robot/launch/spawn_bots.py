import argparse
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import subprocess
def write_yaml_file(num_bots,x_poses,y_poses,z_poses):
    yaml_path = os.path.join(get_package_share_directory('turtlebot3_multi_robot'),'params','robot_data.yaml')
    my_data = dict()
    ns = 'bot_'

    for i in range(num_bots):
        bot_name = ns+str(i)
        my_data[bot_name]=dict()
        my_data[bot_name]['name']=f'{bot_name}'
        my_data[bot_name]['x_pose']=str(x_poses[i])
        my_data[bot_name]['y_pose']=str(y_poses[i])
        my_data[bot_name]['z_pose']=str(z_poses[i])
    with open(yaml_path,"w") as file:
        yaml.dump(my_data,file,default_flow_style=False)

        
        
        
def parse_args():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multiple robots.")
    parser.add_argument('--num_bots', type=int, default=4, help="Number of robots to spawn.")
    parser.add_argument('--initial_pose', type=str, default=None, help="Number of robots to spawn.")
    parser.add_argument('--x_pose', type=str, nargs='+', default=[-2.0, 0.0, 0.0, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.0 , 0.0, 1.0, -1.0 ], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--y_pose', type=str, nargs='+', default=[2.0, 0.5, -0.5, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5, 0.5, -1.0, 0.0, 0.0], 
                        help="List of x positions for the robots (space-separated).")
    parser.add_argument('--z_pose', type=str, nargs='+', default=[0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05], 
                        help="List of x positions for the robots (space-separated).")
    
    args = parser.parse_args()
    
    # Convert x_pose argument into a list of floats
    x_poses = [float(x) for x in args.x_pose]
    y_poses = [float(x) for x in args.y_pose]
    z_poses = [float(x) for x in args.z_pose]
    
    return args.num_bots, x_poses,y_poses,z_poses


if __name__ == '__main__':

    
    num_bots, x_poses, y_poses,z_poses = parse_args()
    write_yaml_file(num_bots, x_poses,y_poses,z_poses)
    launch_file = os.path.join(get_package_share_directory('turtlebot3_multi_robot'),'launch','gazebo_multi_nav2_world_orig.launch.py')
    result = subprocess.run(['ros2', 'launch',launch_file], capture_output=True, text=True)
    print(result)
