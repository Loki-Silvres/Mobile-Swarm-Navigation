from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import launch_ros
def generate_launch_description():

    package_name='turtlebot3_gazebo' #<--- CHANGE ME
    pkgPath=launch_ros.substitutions.FindPackageShare(package='turtlebot3_xacro').find('turtlebot3_xacro')
    world_param = DeclareLaunchArgument('world', default_value='',
                          description='gazebo World')
    rvizConfigPath= os.path.join(pkgPath, 'config/config.rviz')

    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )




    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_car'],
                        output='screen')
    rviz_node=launch_ros.actions.Node(package='rviz2',executable='rviz2',name='rviz2',output='screen',arguments=['-d',rvizConfigPath])


     

    
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),
        world_param,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch'), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments=[('world',LaunchConfiguration('world'))]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch'), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        rsp,
        spawn_entity,
        rviz_node
    ])
