#!/usr/bin/env bash

. /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://localhost:11311
roscore &

. /opt/ros/galactic/setup.bash

ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
