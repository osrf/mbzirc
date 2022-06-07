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

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='mbzirc_seed',
            plugin='mbzirc_seed::UavController',
            namespace='quadrotor_1',
            parameters=[{
                'x_vel': 1.0,
                'y_vel': 1.0,
                'target_pressure': 101000.0
            }]
        ),
        ComposableNode(
            package='mbzirc_seed',
            plugin='mbzirc_seed::UavController',
            namespace='quadrotor_2',
            parameters=[{
                'x_vel': 1.0,
                'y_vel': -1.0,
                'target_pressure': 101000.0
            }]
        ),
        ComposableNode(
            package='mbzirc_seed',
            plugin='mbzirc_seed::UsvController',
            namespace='usv',
        ),
    ]

    container = ComposableNodeContainer(
        name='uav_controllers',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes 
    )

    return LaunchDescription([container])
