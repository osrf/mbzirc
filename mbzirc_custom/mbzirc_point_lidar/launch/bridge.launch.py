# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import mbzirc_ign.payload_bridges


def launch(context, *args, **kwargs):

    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    slot_idx = LaunchConfiguration('slot_idx').perform(context)

    nodes = []

    # create IGN to ROS bridges for lidar scan and point cloud topic
    # using the mbzirc_ign module, see available bridge in
    # mbzirc_ign/src/mbzirc_ign/payload_bridges.py
    bridges = [
        mbzirc_ign.payload_bridges.lidar_scan(world_name, model_name, slot_idx),
        mbzirc_ign.payload_bridges.lidar_points(world_name, model_name, slot_idx)
    ]

    # create a node the bridges
    # The node runs the parameter_bridge executable, see
    # https://github.com/ignitionrobotics/ros_ign/blob/galactic/ros_ign_bridge
    nodes.append(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
        parameters=[{'lazy': True}],
    ))

    # Add model name as namepace to generate unique topic names
    launch_processes = []
    group_action = GroupAction([
        PushRosNamespace(model_name),
        *nodes
    ])

    launch_processes.append(group_action)
    return launch_processes

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments passed in from mbzirc_ign/src/mbzirc_ign/model.py
        # Used for remapping topic names and namespacing ros node
        DeclareLaunchArgument('world_name', default_value='',
            description='Name of world'),
        DeclareLaunchArgument('model_name', default_value='',
            description='Name of model'),
        DeclareLaunchArgument('slot_idx', default_value='',
            description='Index of sensor slot'),
        OpaqueFunction(function = launch)
        ])

