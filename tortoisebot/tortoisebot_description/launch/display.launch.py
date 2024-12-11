import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource





def generate_launch_description():
    
    
    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
    state_publisher_dir = os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
    default_rviz_config_path = os.path.join(get_package_share_directory('tortoisebot_description'), 'rviz/tortoisebot_sensor_display.rviz')
    
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],

    )
    gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())
    
    state_publisher_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(state_publisher_dir, 'state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        
    )    

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        


        state_publisher_nodes,
        gazebo_launch_cmd,
        rviz_node,
    ])
    