#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.events import Shutdown



def generate_launch_description():
    ld = LaunchDescription()
    # initializations
    package_dir = get_package_share_directory('auto_explore')
    launch_dir = os.path.join(package_dir, 'launch')
    nav_bringup_dir = os.path.join(launch_dir, 'bringup')

    
    
    bot_desc= {'name':'amr','x_pose':2.50,'y_pose':0.4,'z_pose':0.01}

    BOT_MODEL = 'waffle_pi'

    urdf = os.path.join(
        package_dir, 'urdf', 'turtlebot3_' + BOT_MODEL + '.urdf'
    )
    
    
    
    
    # If warehouse world has to be simulated, uncomment the following and use the below bot_desc variable, comment the previous one 
    # world = '/home/loki/auto_explore_ws/src/aws-robomaker-small-warehouse-world-ros2/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world'
    # bot_desc = {'name':'amr','x_pose':-2.75,'y_pose':4.50,'z_pose':0.01}
    

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    





    #launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )


    declare_use_sim_time = DeclareLaunchArgument(
    name='use_sim_time', default_value='true', description='Use simulator time'
    )


    

    explore_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')),
        
    )
    
    
    
    

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time)

    ld.add_action(explore_cmd)

    
    
    return ld