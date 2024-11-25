import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def create_launch_description(context, *args, **kwargs):
    """Opaque function that creates and configures the launch description."""
    # Get the URDF file path based on the TURTLEBOT3_MODEL environment variable
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # # model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    # urdf_path = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'models',
    #     model_folder,
    #     'model.sdf'
    # )

    namespace = LaunchConfiguration('namespace').perform(context)
    

    xacro_file = os.path.join(get_package_share_directory('turtlebot3_xacro'), 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"prefix": namespace+'/'})
    robot_desc = doc.toprettyxml()
    urdf_output_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'urdf','my_bot.urdf')
    with open(urdf_output_path, 'w') as urdf_file:
        urdf_file.write(robot_desc)

    my_robot_desc = namespace + '/robot_description'
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.05')
    
    # Declare launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify x-position of the robot in the simulation'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify y-position of the robot in the simulation'
    )
    declare_y_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.05',
        description='Specify y-position of the robot in the simulation'
    )

    # Declare namespace and entity name arguments
    # declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='my_bot')
    declare_entity_name_cmd = DeclareLaunchArgument('entity_name', default_value='waffle')

    # Start Gazebo and spawn the entity (robot)
    # start_gazebo_ros_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', LaunchConfiguration('entity_name'),
    #         '-file', xacro_file,
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', '0.01',
    #         '-robot_namespace', LaunchConfiguration('namespace')
    #     ],
    #     output='screen',
    # )

    print(my_robot_desc)
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('entity_name'),
            '-topic', my_robot_desc,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-robot_namespace', LaunchConfiguration('namespace')
        ],
        output='screen',
    )

    # Return a list of actions to be executed
    return [
        declare_entity_name_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        start_gazebo_ros_spawner_cmd
    ]


def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='my_bot',
            description='Namespace for the robot'
        ),
        OpaqueFunction(function=create_launch_description)
    ]
        )
