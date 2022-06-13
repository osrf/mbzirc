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

def launch(context, *args, **kwargs):

    model_name = LaunchConfiguration('model_name').perform(context)
    slot_idx = LaunchConfiguration('slot_idx').perform(context)

    nodes = []

    # create a node to launch the the custom IGN to ROS bridge executable
    ignTopic = f'/model/{model_name}/model/sensor_{slot_idx}/radar/scan'
    nodes.append(Node(
        package='mbzirc_naive_radar',
        executable='naive_radar_bridge',
        parameters=[{'topic': ignTopic}],
        remappings=[('radar/scan', f'slot{slot_idx}/radar/scan')]))

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
        DeclareLaunchArgument('slot_idx', default_value='',
            description='Index of sensor slot'),
        OpaqueFunction(function = launch)
        ])

