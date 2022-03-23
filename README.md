**This is work in progress and is subject to change in the next releases. Please do not use the information provided in the simulation environment until unless officially announced by the organizers of MBZIRC**


# MBZIRC Maritime Grand Challenge Simulator

This repository contains simulation software created for the
MBZIRC Maritime Grand Challenge.

## Installation

### Prerequsite

* Platform: Ubuntu 20.04 (Focal)
* Ignition Fortress
* ROS2 Galactic

See Installation instructions for:

* Ignition Fortress: https://ignitionrobotics.org/docs/fortress

* ROS2 Galactic:  https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html


### Installation from Source

1. Create a colcon workspace and clone the mbzirc repo

    ```
    mkdir -p ~/mbzirc_ws/src
    cd ~/mbzirc_ws/src
    git clone https://github.com/osrf/mbzirc.git
    ```

1. Clone the `ros_ign` repo and check out the `ros2` branch

    ```
    cd ~/mbzirc_ws/src
    git clone https://github.com/ignitionrobotics/ros_ign.git -b ros2
    ```

1. Install dependencies using [rosdep](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html#installing-and-initializing-rosdep)

    ```
    cd ~/mbzirc_ws
    rosdep update
    rosdep install -r --from-paths src -i -y --rosdistro galactic
    ```

    Make sure the `ros-galactic-mavros-msgs` package is installed, e.g.

    ```
    dpkg -l | grep ros-galactic-mavros-msgs
    ```

    If not, install it:

    ```
    sudo apt install ros-galactic-mavros-msgs
    ```

1. Build the workspace

    ```
    cd ~/mbzirc_ws
    colcon build --merge-install
    ```

### Docker setup

Docker images are available on Docker Hub: https://hub.docker.com/repository/docker/osrf/mbzirc

1. Pull the latest version of the docker image

    ```
    docker pull osrf/mbzirc:mbzirc_sim_latest
    ```

1. Clone the repo and launch a Docker container from the image using the `run.bash` script. Note: requires `nvidia-docker2`

    ```
    git clone https://github.com/osrf/mbzirc.git
    cd mbzirc/docker
    bash run.bash osrf/mbzirc:mbzirc_sim_latest  /bin/bash
    ```

To build a docker image of the simulator locally:


1. Navigate to the `docker` directory and build the `mbzirc_sim` Docker image

    ```
    cd mbzirc
    bash docker/build.bash mbzirc_sim
    ```

1.  The process can take a few minutes. Once it is done, you can launch the
 Docker container:

    ```
    bash run.bash mbzirc_sim
    ```

## Running the simulator

Please see the wiki:  https://github.com/osrf/mbzirc/wiki

