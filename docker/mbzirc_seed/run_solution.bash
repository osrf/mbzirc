#!/usr/bin/env bash

. /opt/ros/galactic/setup.bash
. ~/mbzirc_ws/install/setup.sh

mkdir -p /tmp/ign/logs

export ROS_LOG_DIR=/tmp/ign/logs/ros
unbuffer ros2 launch mbzirc_seed mbzirc_seed.launch.py robot_name:="$@"  2>&1 | tee /home/developer/.ros/solution.log
