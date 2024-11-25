from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to "true" to launch the simulation with a GUI'
    )
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Whether to start the simulation paused'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run in headless mode (no GUI)'
    )
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable debug mode'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='true',
        description='Enable verbose output in Gazebo'
    )

    # Find the Gazebo and world file paths
    gazebo_launch_file = FindPackageShare('gazebo_ros').find('gazebo_ros') + '/launch/gazebo.launch.py'
    # world_file = '/home/pacman/turtlebot3_ws/src/my_world/worlds/warehouse.world'
    world_file = os.path.join(get_package_share_directory('my_world'),'worlds','warehouse.world')
    

    # Include the Gazebo launch file with specified arguments
    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            #'world': world_file,
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'headless': LaunchConfiguration('headless'),
            'debug': LaunchConfiguration('debug'),
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )

    return LaunchDescription([
        gui_arg,
        paused_arg,
        use_sim_time_arg,
        headless_arg,
        debug_arg,
        verbose_arg,
        gazebo_include
    ])
