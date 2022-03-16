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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os

def launch(context, *args, **kwargs):

    ign_args = LaunchConfiguration('ign_args').perform(context)

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_ign_gazebo'), 'launch'),
        '/ign_gazebo.launch.py']),
        launch_arguments = {'ign_args': ign_args}.items())

    ros2_ign_score_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/mbzirc/score@std_msgs/msg/Float32@ignition.msgs.Float'],
    )

    ros2_ign_run_clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/mbzirc/run_clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
    )

    ros2_ign_phase_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/mbzirc/phase@std_msgs/msg/String@ignition.msgs.StringMsg'],
    )

    return [ign_gazebo,
            ros2_ign_score_bridge,
            ros2_ign_run_clock_bridge,
            ros2_ign_phase_bridge]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ign_args', default_value='',
            description='Arguments to be passed to Ignition Gazebo'),
        OpaqueFunction(function = launch)
        ])

