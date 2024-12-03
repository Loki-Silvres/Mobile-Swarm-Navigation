import os
from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('name', 'turtlebot')


def launch_setup():

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    name = sl.arg('name')
    
    with sl.group(ns=name):
        
        sl.robot_state_publisher('turtlebot3_xacro','turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro',
                                 xacro_args={'prefix': name+'/', 'gazebo': False})

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
