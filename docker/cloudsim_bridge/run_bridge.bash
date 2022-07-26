#!/usr/bin/env bash

. /opt/ros/galactic/setup.bash
. ~/mbzirc_ws/install/setup.sh

mkdir -p /tmp/ign/logs

# Optionally stall this bridge from starting to allow configuration files to be populated.
# MBZIRC_STALL_START_PATH contains a path to a file.
# If MBZIRC_STALL_START_PATH is set, stall the bridge from starting until the file exists.
if [ -n "${MBZIRC_STALL_START_PATH}" ]; then
  echo "Waiting for configuration files to be ready."
  echo "Stalling until ${MBZIRC_STALL_START_PATH} exists."

  until [ -f "${MBZIRC_STALL_START_PATH}" ]; do
     sleep 1
  done

  echo "Configurations file ready."
  echo "Proceeding to start simulation."
fi

# atop process monitoring
atop -R -w /tmp/ign/logs/atop_log &

export ROS_LOG_DIR=/tmp/ign/logs/ros
unbuffer ros2 launch mbzirc_ign spawn_config.launch.py sim_mode:=bridge "$@"  2>&1 | tee /home/developer/.ros/bridge.log

