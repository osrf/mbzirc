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

from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import mbzirc_ign.bridges

import os


def simulation(world_name):
    ign_args = ['-v 4', '-r']
    ign_args.append(f'{world_name}.sdf')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch'),
            '/ign_gazebo.launch.py']),
        launch_arguments={'ign_args': ' '.join(ign_args)}.items())
    return [ign_gazebo]


def competition_bridges():
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


def spawn(sim_mode, world_name, models, robot=None):
    if type(models) != list:
        models = [models]

    launch_processes = []
    for model in models:
        if robot and model.model_name != robot:
            continue

        # Script to insert model in running simulation
        if sim_mode == 'full' or sim_mode == 'sim':
            ignition_spawn_entity = Node(
                package='ros_ign_gazebo',
                executable='create',
                output='screen',
                arguments=model.spawn_args()
            )
            launch_processes.append(ignition_spawn_entity)

        if sim_mode == 'full' or sim_mode == 'bridge':
            bridges, nodes = model.bridges(world_name)

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

            if sim_mode == 'full':
                handler = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=ignition_spawn_entity,
                        on_exit=[group_action],
                    )
                )
                launch_processes.append(handler)
            elif sim_mode == 'bridge':
                launch_processes.append(group_action)
    return launch_processes
