#!/usr/bin/env bash

# remove previous screens
screen -ls | grep '(Detached)' | awk '{print $1}' | xargs -I % -t screen -X -S % quit

cmd="ros2 launch turtlebot3_xacro bringup_launch.py ${@}"

# test if -d is given
discovery=0

for arg in "$@"
do
  if [ $arg == "-d" ]; then
    discovery=1
  fi
done

if [ $discovery -gt 0 ]; then
    echo "Using discovery server @ $(hostname).local:11811"
    screen -S discovery -dm bash -c 'fastdds discovery --server-id 0'
    sleep 2.
    cmd="ROS_DISCOVERY_SERVER=127.0.0.1:11811 $(echo "$cmd")"
fi

screen -S bringup -dm bash -c "$cmd"
