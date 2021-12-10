# MBZIRC Maritime Grand Challenge Simulator

This repository contains information on the simulation platform built for the
MBZIRC Maritime Grand Challenge.

## Installation


### Prerequsite

Platform support for the MBZIRC Maritime Grand Challenge Simulator is
Ignition Fortress and Galactic on Ubuntu 20.04 (Focal).

See Installation instructions for:

* Ignition Fortress: https://ignitionrobotics.org/docs/fortress

* ROS2 Galactic:  https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html


### Installation from Source

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

#### Run the demo

1. Source the setup file

  ```
  cd ~/mbzirc_ws
  source install/share/setup.bash
  ```

1. If you are building on Ubuntu BioniSet the environment path (optional)

  ```
  export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:install/share/mbzirc_ign/models:install/share/mbzirc_ign/worlds
  ```

1. Launch simple demo world

  ```
  ign gazebo -v 4 simple_demo.sdf
  ```


### Installation using Docker

1. Navigate to the `docker` directory and build the `mbzirc_sim` docker image

  ```
  bash build.bash mbzirc_sim
  ```

#### Run the demo

  ```
  bash run.bash mbzirc_sim
  ```
