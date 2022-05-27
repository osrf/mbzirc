# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from mbzirc_ign.model import Model

import mbzirc_ign.bridges

import os


def spawn(context, config_file, world_name):
    with open(config_file, 'r') as stream:
        models = Model.FromConfig(stream)

    if type(models) != list:
        # In the case that a single model is parsed, pack it in a list
        models = [models]

    launch_processes = []
    bridges = []
    nodes = []

    # Only one instance of compeititon topics required
    launch_processes.extend(launch_competition_bridges())

    for model in models:
        ignition_spawn_entity = Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=model.spawn_args()
        )
        launch_processes.append(ignition_spawn_entity)

        bridges, nodes = model.bridges(world_name)

        if model.isUAV():
            payload = model.payload_bridges(world_name)
            payload_bridges = payload[0]
            payload_nodes = payload[1]
            payload_launches = payload[2]
            bridges.extend(payload_bridges)
            nodes.extend(payload_nodes)
            launch_processes.extend(payload_launches)

        if model.isFixedWingUAV():
            nodes.append(Node(
                package='mbzirc_ros',
                executable='fixed_wing_bridge',
                output='screen',
                parameters=[{'model_name': model.model_name}],
            ))

        nodes.append(Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[bridge.argument() for bridge in bridges],
            remappings=[bridge.remapping() for bridge in bridges],
        ))

        # tf broadcaster
        nodes.append(Node(
            package='mbzirc_ros',
            executable='pose_tf_broadcaster',
            output='screen',
            parameters=[
                {'world_frame': world_name}
            ]
        ))

        # video target relay
        nodes.append(Node(
            package='mbzirc_ros',
            executable='video_target_relay',
            output='screen',
            parameters=[{'model_name': model.model_name}]
        ))

        group_action = GroupAction([
            PushRosNamespace(model.model_name),
            *nodes
        ])

        handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[group_action],
            )
        )
        launch_processes.append(handler)
    return launch_processes


def launch_competition_bridges():
    bridges = [
        mbzirc_ign.bridges.score(),
        mbzirc_ign.bridges.clock(),
        mbzirc_ign.bridges.run_clock(),
        mbzirc_ign.bridges.phase(),
        mbzirc_ign.bridges.stream_status(),
    ]
    nodes = []
    nodes.append(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))
    return nodes


def launch_vehicles(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    return spawn(context, config_file, world_name)


def launch_simulation(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    ign_args = ['-v 4', '-r']
    ign_args.append(f'{world_name}.sdf')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_ign_gazebo'), 'launch'),
        '/ign_gazebo.launch.py']),
        launch_arguments = {'ign_args': ' '.join(ign_args)}.items())
    return [ign_gazebo]


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='simple_demo',
            description='Name of world'),
        DeclareLaunchArgument(
            'config_file',
            description='YAML configuration file to spawn'),
        OpaqueFunction(function=launch_simulation),
        # launch setup
        OpaqueFunction(function=launch_vehicles),
    ])
