#!/usr/bin/env bash

. /opt/ros/galactic/setup.bash
. ~/mbzirc_ws/install/setup.sh

mkdir -p /tmp/ign/logs

# atop process monitoring
atop -R -w /tmp/ign/logs/atop_log &

export ROS_LOG_DIR=/tmp/ign/logs/ros
unbuffer ros2 launch mbzirc_ign spawn_config.launch.py sim_mode:=bridge "$@"  2>&1 | tee /home/developer/.ros/bridge.log

