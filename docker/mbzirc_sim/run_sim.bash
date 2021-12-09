#!/usr/bin/env bash

. /opt/ros/galactic/setup.bash
. ~/mbzirc_ws/install/setup.sh

mkdir -p /tmp/ign/logs

export ROS_LOG_DIR=/tmp/ign/logs/ros
unbuffer ign gazebo -v 4 "$@" 2>&1 | tee /tmp/ign/logs/gzserver_stdout.log
