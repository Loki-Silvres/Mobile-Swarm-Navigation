import os
from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time=True)

sl.declare_arg('name', 'turtlebot')
sl.declare_gazebo_axes(x = 0., y = 0., z = .3, yaw = 0.)
sl.declare_arg('gui', False, description = 'If we want sliders to control the velocity manually')
sl.declare_arg('gt', False, description = 'If Gazebo should publish the odometry or the ground truth')


def launch_setup():

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    name = sl.arg('name')
    
    with sl.group(ns=name):
        
        sl.robot_state_publisher('turtlebot3_xacro','turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro',
                                 xacro_args={'prefix': name+'/', 'gazebo': True})

        sl.spawn_gz_model(name, spawn_args = [sl.gazebo_axes_args()])
        
        bridges = []
        # joint states
        gz_js_topic = GazeboBridge.model_prefix(name)+'/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        if sl.arg('gt'):
            bridges.append(GazeboBridge(f'/model/{name}/pose',
                                     'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
            sl.node('pose_to_tf', parameters={'child_frame': name + '/base_footprint'})
        else:
            bridges.append(GazeboBridge(f'/model/{name}/odometry',
                            'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros, gz_msg='gz.msgs.Odometry'))
            bridges.append(GazeboBridge(f'/model/{name}/tf',
                        '/tf', 'tf2_msgs/msg/TFMessage', GazeboBridge.gz2ros))
        
        bridges.append(GazeboBridge(GazeboBridge.model_topic(name, 'cmd_vel'),
                                     'cmd_vel', 'geometry_msgs/Twist', GazeboBridge.ros2gz))

        bridges.append(GazeboBridge(name+'/imu', 'imu', 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        bridges.append(GazeboBridge(name+'/image', 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))

        # LiDAR
        bridges.append(GazeboBridge(name+'/scan/points', 'scan', 'sensor_msgs/LaserScan', GazeboBridge.gz2ros))
        bridges.append(GazeboBridge(name+'/scan', 'scan', 'sensor_msgs/LaserScan', GazeboBridge.gz2ros))

        sl.create_gz_bridge(bridges)
        
        if sl.arg('gui'):
            sl.node('slider_publisher', arguments=[sl.find('turtlebot3_xacro', 'Twist.yaml')])

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
