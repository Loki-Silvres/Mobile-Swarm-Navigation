#!/usr/bin/bash

if [[ $# -eq 0 ]]; then
    echo "Give a destination (e.g. hallS, amphiE)"
else
    echo "Begin ros2 bag record to $1"
    ns=$(hostname)
    ros2 bag record $ns/cmd_vel $ns/joint_states $ns/imu $ns/odom $ns/scan -o $1
fi

