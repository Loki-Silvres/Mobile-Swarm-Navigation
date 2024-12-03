from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node,PushRosNamespace


from launch_ros.actions import PushRosNamespace
def generate_launch_description():
    # Declare launch arguments (you can customize these values)
    return LaunchDescription([

        # Push the namespaces to ROS arguments
        PushRosNamespace('bot_0'),

        # Log the arguments to verify the launch file is being processed
        LogInfo(
            condition=None,
            msg="Launching costmap node with given namespace and arguments."
        ),

        # Launch the node with NodeOptions arguments
        Node(
            package='nav2_costmap_2d',  # Replace with your package name
            executable='nav2_costmap_2d',  # Replace with your executable name
            name='my_costmap_node',  # This will be passed as __node:=my_costmap_node
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Set the use_sim_time parameter directly
            }],
            remappings=[  # Remapping if required
                # ('/map', '/custom_map'),
                ('/scan', '/bot_0/scan')
            ],
            # arguments=[
            #     '--ros-args', 
            #     '-r', 'parent_namespace:=parent',  # Passing namespaces
            #     '-r', 'local_namespace:=local',
            #     '-r', 'name:=my_costmap_node',  # Setting the node name
            #     '-p', 'use_sim_time:=false',  # Set the simulation time flag
            # ]
        ),
    ])
