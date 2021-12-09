# MBZIRC Maritime Grand Challenge Simulator

This repository contains information on the simulation platform built for the
MBZIRC Maritime Grand Challenge.

## Installation

1. Create a colcon workspace and clone the repo

  ```
  mkdir -p ~/mbzirc_ws/src
  cd ~/mbzirc_ws/src
  git clone https://github.com/osrf/mbzirc.git
  ```

1. Build the workspace

  ```
  cd ~/mbzirc_ws
  colcon build --merge-install
  ```

## Run the demo

1. Source the setup file

  ```
  cd ~/mbzirc_ws
  source install/share/setup.bash
  ```

1. Set the environment path

  ```
  export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:install/share/mbzirc_ign/models:install/share/mbzirc_ign/worlds
  ```

1. Launch simple demo world

  ```
  ign gazebo -v 4 simple_demo.sdf
  ```


