#!/usr/bin/env python3

from simple_launch import SimpleLauncher
import os


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time=False)
    name = sl.declare_arg('name', os.uname().nodename)

    with sl.group(ns=name):
    
        sl.node('v4l2_camera', 'v4l2_camera_node',
                parameters = {'width': 640, 'height': 480,
                                'camera_info_url': 'file://'+sl.find('turtlebot3_xacro', 'pi_camera.yaml')})

    return sl.launch_description()
