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

1. Launch simple demo world

    ```
    ros2 launch ros_ign_gazebo ign_gazebo.launch.py ign_args:="-v 4 -r simple_demo.sdf"
    ```

  * This is equivalent to:

    ```
    ign gazebo -v 4 simple_demo.sdf
    ```

    * You may have to set the environment variables:

      ```
      export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:install/share/mbzirc_ign/models:install/share/mbzirc_ign/worlds
      ```

1. In a separate terminal, spawn a UAV

    ```
    # remember to source the setup.bash
    source install/share/setup.bash

    ros2 launch mbzirc_ign spawn.launch.py name:=x4 world:=simple_demo model:=x4 type:=uav x:=1 y:=2 z:=0.08 R:=0 P:=0 Y:=0
    ```

1. In another terminal, you can take a look at the ROS2 topics available

    ```
    ros2 topic list
    ```

1. Make sure data are published, e.g. try echoing the IMU topic

    ```
    ros2 topic echo /x4/imu/data
    ```

1. Launch `rqt_image_view` to see camera stream from the UAV

    ```
    ros2 run rqt_image_view rqt_image_view &
    ```

1. Publish a twist command with linear +z velocity to make the UAV take off

    ```
    ros2 topic pub --once /x4/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0,y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    ```

1. Publish zero velocity twist command to make the UAV hover in the air

    ```
    ros2 topic pub --once /x4/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    ```

1. View the TF tree

    ```
    # this generates frames.pdf
    ros2 run tf2_tools view_frames

    # view the TF tree using your favorite PDF viewer, e.g.
    evince frames.pdf
    ```

1. Visualize point clouds in rviz2

   ```
   ros2 run rviz2 rviz2
   ```

   Set `Fixed Frame` to `/x4/base_link`

   Click on `Add` button below the left panel, select the `By topic` tab and choose the `/x4/rgbd_camera/depth/points` > `PointCloud2` topic to see colored point clouds in the 3D view.


1. In a separate terminal, spawn a USV

    ```
    # remember to source the setup.bash
    source install/share/setup.bash

    ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=wam-v type:=usv x:=0 y:=0 z:=1.0 R:=3.14 P:=0 Y:=0
    ```

1. To move the propeller
    ```
    ign topic -t /model/usv/joint/left_engine_propeller_joint/cmd_thrust -m ignition.msgs.Double -p 'data: -1'
    ign topic -t /model/usv/joint/right_engine_propeller_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 1'
    ```

1. To rotate the thruster
    ```
    ign topic -t /model/usv/joint/left_chasis_engine_joint/0/cmd_pos -m ignition.msgs.Double -p 'data: -1'
    ign topic -t /model/usv/joint/right_chasis_engine_joint/0/cmd_pos -m ignition.msgs.Double -p 'data: -1'
    ```

### Build a Docker image

1. Navigate to the `docker` directory and build the `mbzirc_sim` docker image

    ```
    cd ~/mbzirc_ws/src/mbzirc/docker
    bash build.bash mbzirc_sim
    ```

1.  The process can take a few minutes. Once it is done, you can launch the
 Docker container:

    ```
    bash run.bash mbzirc_sim
    ```
