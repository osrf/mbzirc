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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from mbzirc_ign.model import Model


def spawn(context, config_file, world_name):
    with open(config_file, 'r') as stream:
        m = Model.FromConfig(stream)

    if type(m) == list:
        print('Spawn Config only supports one model currently')
        model = m[0]
    else:
        model = m

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=model.spawn_args()
    )

    nodes = []
    bridges = model.bridges(world_name)

    if model.isUAV():
        [payload_bridges, payload_nodes] = model.payload_bridges(world_name)
        bridges.extend(payload_bridges)
        nodes.extend(payload_nodes)

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
    return [ignition_spawn_entity, handler]


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    return spawn(context, config_file, world_name)


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

        # launch setup
        OpaqueFunction(function=launch)
    ])
