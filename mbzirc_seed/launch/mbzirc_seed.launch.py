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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from dataclasses import dataclass, field

@dataclass(frozen=True)
class RobotConfig:
    '''
    Convenience representation of a robot configuration
    '''
    name: str
    plugin: str
    parameters: dict = field(default_factory=dict)

    def node(self):
        return ComposableNode(
                package='mbzirc_seed',
                plugin=f'mbzirc_seed::{self.plugin}',
                namespace=self.name,
                parameters=[self.parameters])


# Enumerate all available robots
# These should match the available platforms in the team config yaml
ROBOTS = [
    RobotConfig(
        name='quadrotor_1',
        plugin='UavController',
        parameters={
            'x_vel': 1.0,
            'y_vel': 1.0,
            'target_pressure': 101000.0
        }
    ),
    RobotConfig(
        name='quadrotor_2',
        plugin='UavController',
        parameters={
            'x_vel': 1.0,
            'y_vel': 1.0,
            'target_pressure': 101000.0
        }
    ),
    RobotConfig(
        name='usv',
        plugin='UsvController',
    )
]


def launch(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    # If robot_name is empty, return all, otherwise match
    nodes = filter(lambda r: robot_name == '' or r.name == robot_name, ROBOTS)

    container = ComposableNodeContainer(
        name='controllers',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[n.node() for n in nodes]
    )
    return [container]

def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Robot name to launch'),
        OpaqueFunction(function=launch),
    ])
