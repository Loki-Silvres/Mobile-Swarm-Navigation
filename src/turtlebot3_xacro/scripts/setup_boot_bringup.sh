#!/usr/bin/bash

tbot_current=$(crontab -l | grep 'bringup.sh' -wc)

if [[ -z $tbot_current ]]; then
    echo "ROS 2 stack already in crontab"
else
    echo "Adding ROS 2 stack to crontab"
    THIS_SCRIPT=$(readlink -f $0)
    THIS_DIR=`dirname $THIS_SCRIPT`
    (crontab -l 2>/dev/null; echo "@reboot $THIS_DIR/bringup.sh") | crontab -
fi
