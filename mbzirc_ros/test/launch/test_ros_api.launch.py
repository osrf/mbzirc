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

import unittest

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import launch_testing

# launch simple_demo and spawn quadrotor and hexrotor UAVs
def generate_test_description():

    process_under_test = Node(
        package='mbzirc_ros',
        executable='test_ros_api',
        output='screen'
    )

    # launch simple_demo world
    gazebo = ExecuteProcess(
        cmd=['ign gazebo -v 4 --iterations 20000 -s -r simple_demo.sdf'],
        output='screen',
        shell=True
    )

    display_test = os.getenv('DISPLAY_TEST')
    # spawn quadrotor
    arguments={'name'  : 'quadrotor',
               'world' : 'simple_demo',
               'model' : 'mbzirc_quadrotor',
               'type'  : 'uav',
               'z'     : '0.08',}
    # if display_test == '1':
    arguments['slot0'] = 'mbzirc_hd_camera'

    spawn_quadrotor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mbzirc_ign'),
                'launch/spawn.launch.py')
        ),
        launch_arguments=arguments.items())
    delay_launch_quadrotor = TimerAction(
            period=10.0,
            actions=[spawn_quadrotor])

    # spawn hexrotor
    arguments={'name'  : 'hexrotor',
               'world' : 'simple_demo',
               'model' : 'mbzirc_hexrotor',
               'type'  : 'uav',
               'x'     : '2',
               'z'     : '0.08',}
    # if display_test == '1':
    #     arguments['slot0'] = 'mbzirc_rgbd_camera'
    spawn_hexrotor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mbzirc_ign'),
                'launch/spawn.launch.py')
        ),
        launch_arguments=arguments.items())
    delay_launch_hexrotor = TimerAction(
            period=13.0,
            actions=[spawn_hexrotor])

    return LaunchDescription([
        gazebo,
        delay_launch_quadrotor,
        # delay_launch_hexrotor,
        process_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class RosApiTest(unittest.TestCase):

    def test_termination(self, process_under_test, gazebo, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=300)
        proc_info.assertWaitForShutdown(process=gazebo, timeout=300)


@launch_testing.post_shutdown_test()
class RosApiTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )