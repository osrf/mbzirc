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

import mbzirc_ign.bridges

def launch(context, *args, **kwargs):

    model_name = LaunchConfiguration('model_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)

    nodes = []
    bridges = []

    # arm joint states
    bridges.append(
        mbzirc_ign.bridges.arm_joint_states(world_name, model_name)
    )

    # arm joint pos cmd
    arm_joints = ['joint_0', 'joint_1', 'joint_2']
    for joint in arm_joints:
        bridges.append(
            mbzirc_ign.bridges.arm_joint_pos(model_name, joint)
        )

    # create a node to bridge IGN <-> ROS arm joint states and pos cmd topics
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
        DeclareLaunchArgument('World name', default_value='',
            description='Name of world'),
        DeclareLaunchArgument('model_name', default_value='',
            description='Name of model'),
        OpaqueFunction(function = launch)
        ])

