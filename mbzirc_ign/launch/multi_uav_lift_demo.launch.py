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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/spawn.launch.py']),
            launch_arguments={
                'name': 'quadrotor_1',
                'world': 'empty_platform',
                'model': 'mbzirc_quadrotor',
                'x': '0',
                'y': '0.5',
                'z': '0',
                'gripper': 'mbzirc_suction_gripper'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/spawn.launch.py']),
            launch_arguments={
                'name': 'quadrotor_2',
                'world': 'empty_platform',
                'model': 'mbzirc_quadrotor',
                'x': '0',
                'y': '-0.5',
                'z': '0',
                'gripper': 'mbzirc_suction_gripper'
            }.items(),
        )
    ])
