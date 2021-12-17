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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
                        #, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import subprocess
import codecs

def spawn_uav(context, model_path, world_name, model_name, link_name):

  x_pos = LaunchConfiguration('x').perform(context)
  y_pos = LaunchConfiguration('y').perform(context)
  z_pos = LaunchConfiguration('z').perform(context)
  r_rot = LaunchConfiguration('R').perform(context)
  p_rot = LaunchConfiguration('P').perform(context)
  y_rot = LaunchConfiguration('Y').perform(context)

  model_file = os.path.join(
      get_package_share_directory('mbzirc_ign'), 'models', model_path, 'model.sdf.erb')
  print("spawning UAV file: " + model_file)

  # run erb
  process = subprocess.Popen(['erb', 'name=' + model_name, model_file], stdout=subprocess.PIPE)
  stdout = process.communicate()[0]
  str_output = codecs.getdecoder("unicode_escape")(stdout)[0]

  ignition_spawn_entity = Node(
      package='ros_ign_gazebo',
      executable='create',
      output='screen',
      arguments=['-string', str_output,
                 '-name', model_name,
                 '-allow_renaming', 'false',
                 '-x', x_pos,
                 '-y', y_pos,
                 '-z', z_pos,
                 '-R', r_rot,
                 '-P', p_rot,
                 '-Y', y_rot,
                ],
  )

  sensor_prefix = '/world/' + world_name + '/model/' + model_name + '/link/' + link_name + '/sensor'

  # imu
  ros2_ign_imu_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=[sensor_prefix +  '/imu_sensor/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
      remappings=[(sensor_prefix + "/imu_sensor/imu", 'imu/data')]
  )

  # magnetometer
  ros2_ign_magnetometer_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=[sensor_prefix +  '/magnetometer/magnetometer@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer'],
      remappings=[(sensor_prefix + '/magnetometer/magnetometer', 'magnetic_field')]
  )

  # air pressure
  ros2_ign_air_pressure_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=[sensor_prefix +  '/air_pressure/air_pressure@sensor_msgs/msg/FluidPressure@ignition.msgs.FluidPressure'],
      remappings=[(sensor_prefix + '/air_pressure/air_pressure', 'air_pressure')]
  )

  # camera - image transport
  # ros2_ign_camera_bridge = Node(
  #     package='ros_ign_image',
  #     executable='image_bridge',
  #     output='screen',
  #     arguments=[sensor_prefix +  '/camera_front/image'],
  #     remappings=[(sensor_prefix + '/camera_front/image', 'front/image_raw')]
  # )

  # camera - parameter bridge
  ros2_ign_camera_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      arguments=[sensor_prefix +  '/camera_front/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                 sensor_prefix +  '/camera_front/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
      remappings=[(sensor_prefix + '/camera_front/image', 'front/image_raw'),
                  (sensor_prefix + '/camera_front/camera_info', 'front/camera_info')]
  )

  # camera optical frame publisher
  ros2_camera_optical_frame_publisher = Node(
      package='mbzirc_ros',
      executable='optical_frame_publisher',
      arguments=['1'],
      remappings=[('input/image', 'front/image_raw'),
                  ('output/image', 'front/optical/image_raw'),
                  ('input/camera_info', 'front/camera_info'),
                  ('output/camera_info', 'front/optical/camera_info'),
                 ]
  )

  # lidar
  ros2_ign_lidar_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      arguments=[sensor_prefix +  '/front_laser/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
      remappings=[(sensor_prefix + '/front_laser/scan/points', 'points')]
  )

  # twist
  ros2_ign_twist_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=['/model/' + model_name + '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
      remappings=[('/model/' + model_name +'/cmd_vel', 'cmd_vel')]
  )

  # pose
  ros2_ign_pose_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=['/model/' + model_name + '/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
      remappings=[('/model/' + model_name +'/pose', 'pose')]
  )

  # pose static
  ros2_ign_pose_static_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=['/model/' + model_name + '/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
      remappings=[('/model/' + model_name +'/pose_static', 'pose_static')]
  )


  # tf broadcaster
  ros2_tf_broadcaster = Node(
      package='mbzirc_ros',
      executable='pose_tf_broadcaster',
      output='screen',
  )

  group_action = GroupAction([
        PushRosNamespace(model_name),
        ros2_ign_imu_bridge,
        ros2_ign_magnetometer_bridge,
        ros2_ign_air_pressure_bridge,
        ros2_ign_camera_bridge,
        ros2_camera_optical_frame_publisher,
        ros2_ign_lidar_bridge,
        ros2_ign_twist_bridge,
        ros2_ign_pose_bridge,
        ros2_ign_pose_static_bridge,
        ros2_tf_broadcaster,
  ])

  handler = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=ignition_spawn_entity,
          on_exit=[group_action],
      ))

  return [ignition_spawn_entity, handler]

def spawn_usv(context, model_path, model_name):

  x_pos = LaunchConfiguration('x').perform(context)
  y_pos = LaunchConfiguration('y').perform(context)
  z_pos = LaunchConfiguration('z').perform(context)
  r_rot = LaunchConfiguration('R').perform(context)
  p_rot = LaunchConfiguration('P').perform(context)
  y_rot = LaunchConfiguration('Y').perform(context)

  model_file = os.path.join(
      get_package_share_directory('mbzirc_ign'), 'models', model_path, 'model.sdf')
  print("spawning USV file: " + model_file)

  ignition_spawn_entity = Node(
      package='ros_ign_gazebo',
      executable='create',
      output='screen',
      arguments=['-file', model_file,
                 '-name', model_name,
                 '-allow_renaming', 'false',
                 '-x', x_pos,
                 '-y', y_pos,
                 '-z', z_pos,
                 '-R', r_rot,
                 '-P', p_rot,
                 '-Y', y_rot,
                ],
  )
  return [ignition_spawn_entity]


def launch(context, *args, **kwargs):
  robot_name = LaunchConfiguration('name').perform(context)
  model_path = LaunchConfiguration('model').perform(context)
  robot_type = LaunchConfiguration('type').perform(context)
  world_name = LaunchConfiguration('world').perform(context)
  if robot_type == 'uav':
      link_name = 'base_link'
      return spawn_uav(context, model_path, world_name, robot_name, link_name)
  elif robot_type == 'usv':
      return spawn_usv(context, model_path, robot_name)



def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'name',
            default_value='',
            description='Name of robot to spawn'),
        DeclareLaunchArgument(
            'world',
            default_value='simple_demo',
            description='Name of world'),
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='SDF model to spawn'),
        DeclareLaunchArgument(
            'type',
            default_value='uav',
            description="type of model to spawn: ['uav', 'usv', 'vessel']"),
        DeclareLaunchArgument(
            'x',
            default_value='0',
            description='X position to spawn'),
        DeclareLaunchArgument(
            'y',
            default_value='0',
            description='y position to spawn'),
        DeclareLaunchArgument(
            'z',
            default_value='0',
            description='z position to spawn'),
        DeclareLaunchArgument(
            'R',
            default_value='0',
            description='R rotation to spawn'),

        DeclareLaunchArgument(
            'P',
            default_value='0',
            description='P rotation to spawn'),
        DeclareLaunchArgument(
            'Y',
            default_value='0',
            description='Y rotation to spawn'),
        # launch setup
        OpaqueFunction(function = launch)
    ])
