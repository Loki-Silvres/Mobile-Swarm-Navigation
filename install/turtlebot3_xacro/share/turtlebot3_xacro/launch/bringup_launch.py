#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim
#
# adapted to multi-robot + simple_launch, Olivier Kermorgant

import os
from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

sl = SimpleLauncher(use_sim_time=False)
sl.declare_arg('name', os.uname().nodename)
sl.declare_arg('cam', True, description='Run the camera node')
sl.declare_arg('odom', True, description='Use improved odom and velocity scaling')
sl.declare_arg('usb_port', default_value='/dev/ttyACM0', description='Connected USB port with OpenCR')
sl.declare_arg('imu', False, description='Use the IMU (differential magneto for basic node, EKF with custom one')
sl.declare_arg('rsp', True, description='Run the robot_state_publisher')


def launch_setup():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    # all happens in this namespace
    name = sl.arg('name')

    custom_odom = sl.arg('odom')
    node_imu = sl.arg('imu') and not custom_odom
    rsp = sl.arg('rsp') or (custom_odom and sl.arg('imu'))

    with sl.group(ns=name):
        
        if rsp:
            sl.robot_state_publisher('turtlebot3_xacro','turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro',
                                    xacro_args={'prefix': name+'/'})
        
        sl.node('hls_lfcd_lds_driver', 'hlds_laser_publisher',
            parameters={'port': '/dev/ttyUSB0', 'frame_id': f'{name}/base_scan'},
            output='screen')

        configured_params = RewrittenYaml(
            source_file=sl.find('turtlebot3_node', TURTLEBOT3_MODEL + '.yaml'),
            root_key=name,
            param_rewrites={'frame_id': f'{name}/odom',
                            'child_frame_id': f'{name}/base_footprint',
                            'use_imu': str(node_imu),
                            'publish_tf': str(not custom_odom)},
            convert_types=True)

        node_remappings = {}
        odom_remappings = {}
        if custom_odom:
            # run bug-free odom and cmd
            if sl.arg('imu'):
                odom_remappings = {'odom': 'odom_raw'}

                ekf_params = RewrittenYaml(
                    source_file = sl.find('turtlebot3_xacro', 'ekf.yaml'),
                    root_key = name,
                    param_rewrites={'odom_frame': f'{name}/odom',
                                    'base_link_frame': f'{name}/base_footprint'},
                    convert_types=True)

                sl.node('robot_localization', 'ekf_node',
                    parameters=[ekf_params],
                    remappings={'odometry/filtered': 'odom'})

            try:
                get_package_share_directory('turtlebot3_odom')
                pkg, node = 'turtlebot3_odom', 'odometry'
            except PackageNotFoundError:
                pkg, node = 'turtlebot3_xacro', 'odometry.py'

            sl.node(pkg, node,
                parameters = {'wheels.max_vel': 0.26,
                              'odom.frame_id': f'{name}/odom',
                              'odom.child_frame_id': f'{name}/base_footprint'},
                remappings = odom_remappings)

            node_remappings = {'cmd_vel': 'cmd_vel_scaled', 'odom': 'odom_wrong'}

        sl.node('turtlebot3_node', 'turtlebot3_ros',
                parameters=[configured_params, {}],
                arguments=['-i', sl.arg('usb_port')],
                remappings = node_remappings)

    if sl.arg('cam'):
        sl.include('turtlebot3_xacro', 'cam_launch.py', launch_arguments={'name': name})

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
