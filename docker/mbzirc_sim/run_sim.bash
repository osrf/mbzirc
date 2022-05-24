#!/usr/bin/env bash

. /opt/ros/galactic/setup.bash
. ~/mbzirc_ws/install/setup.sh

mkdir -p /tmp/ign/logs

# atop process monitoring
atop -R -w /tmp/ign/logs/atop_log &

export ROS_LOG_DIR=/tmp/ign/logs/ros
# todo(iche033) update this to use a new launch file that laucnhes sim and
# spawns all robots using spawn_config
unbuffer ros2 launch ros_ign_gazebo ign_gazebo.launch.py ign_args:="-v 4 -r $@" 2>&1 | tee /tmp/ign/logs/gzserver_stdout.log
