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
  # take flight time in minutes
  flight_time = LaunchConfiguration('flightTime').perform(context)

  slot0_payload = LaunchConfiguration('slot0').perform(context)
  slot1_payload = LaunchConfiguration('slot1').perform(context)
  slot2_payload = LaunchConfiguration('slot2').perform(context)
  slot3_payload = LaunchConfiguration('slot3').perform(context)

  slot0_rpy = LaunchConfiguration('slot0_rpy').perform(context)
  slot1_rpy = LaunchConfiguration('slot1_rpy').perform(context)
  slot2_rpy = LaunchConfiguration('slot2_rpy').perform(context)
  slot3_rpy = LaunchConfiguration('slot3_rpy').perform(context)

  # calculate battery capacity from time
  # capacity (Ah) = flight time (in hours) * load (watts) / voltage
  # assume constant voltage for battery to keep things simple for now.
  battery_capacity = (float(flight_time) / 60) *  6.6 / 12.694

  model_file = os.path.join(
      get_package_share_directory('mbzirc_ign'), 'models', model_path, 'model.sdf.erb')
  print("spawning UAV file: " + model_file)

  # run erb
  command = ['erb']
  command.append(f'name={model_name}')
  command.append(f'capacity={battery_capacity}')

  if slot0_payload: command.append(f'slot0={slot0_payload}')
  if slot0_rpy: command.append(f'slot0_pos={slot0_rpy}')
  if slot1_payload: command.append(f'slot1={slot1_payload}')
  if slot1_rpy: command.append(f'slot1_pos={slot1_rpy}')
  if slot2_payload: command.append(f'slot2={slot2_payload}')
  if slot2_rpy: command.append(f'slot2_pos={slot2_rpy}')
  if slot3_payload: command.append(f'slot3={slot3_payload}')
  if slot3_rpy: command.append(f'slot3_pos={slot3_rpy}')

  command.append(model_file)

  process = subprocess.Popen(command, stdout=subprocess.PIPE)
  stdout = process.communicate()[0]
  str_output = codecs.getdecoder("unicode_escape")(stdout)[0]

  print(command, str_output)

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

  payloads = []
  check = [slot0_payload, slot1_payload, slot2_payload, slot3_payload]

  for idx in range(0, 4):
      payload = check[idx]
      if len(payload) == 0:
          continue
      prefix = f'/world/{world_name}/model/{model_name}/model/sensor_{idx}/link/sensor_link/sensor'

      if payload in ['mbzirc_vga_camera', 'mbzirc_hd_camera']:
          camera_bridge = Node(
              package='ros_ign_bridge',
              executable='parameter_bridge',
              arguments=[prefix +  '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                         prefix +  '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
              remappings=[(prefix + '/camera/image', f'slot{idx}/image_raw'),
                          (prefix + '/camera/camera_info', f'slot{idx}/camera_info')]
          )

          # camera optical frame publisher
          ros2_camera_optical_frame_publisher = Node(
              package='mbzirc_ros',
              executable='optical_frame_publisher',
              arguments=['1'],
              remappings=[('input/image',  f'slot{idx}/image_raw'),
                          ('output/image', f'slot{idx}/optical/image_raw'),
                          ('input/camera_info', f'slot{idx}/camera_info'),
                          ('output/camera_info', f'slot{idx}/optical/camera_info'),
                         ]
          )

          payloads.append(camera_bridge)
          payloads.append(ros2_camera_optical_frame_publisher)
      elif payload in ['mbzirc_planar_lidar', 'mbzirc_3d_lidar']:
          ros2_ign_lidar_bridge = Node(
              package='ros_ign_bridge',
              executable='parameter_bridge',
              output='screen',
              arguments=[prefix + '/lidar/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
              remappings=[(prefix + '/lidar/scan/points', f'slot{idx}/points')]
          )
          payloads.append(ros2_ign_lidar_bridge)
      elif payload in ['mbzirc_rgbd_camera']:
          rgbd_pointcloud_bridge = Node(
              package='ros_ign_bridge',
              executable='parameter_bridge',
              output='screen',
              arguments=[
                  prefix + '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                  prefix + '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                  prefix + '/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                  prefix + '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
              ],
              remappings=[
                  (prefix + '/camera/image', f'slot{idx}/image_raw'),
                  (prefix + '/camera/points', f'slot{idx}/points'),
                  (prefix + '/camera/camera_info', f'slot{idx}/camera_info'),
                  (prefix + '/camera/depth_image', f'slot{idx}/depth_image'),
              ]
          )

          image_optical_frame = Node(
              package='mbzirc_ros',
              executable='optical_frame_publisher',
              arguments=['1'],
              remappings=[('input/image',  f'slot{idx}/image_raw'),
                          ('output/image', f'slot{idx}/optical/image_raw'),
                          ('input/camera_info', f'slot{idx}/camera_info'),
                          ('output/camera_info', f'slot{idx}/optical/camera_info'),
                         ]
          )

          depth_image_optical_frame = Node(
              package='mbzirc_ros',
              executable='optical_frame_publisher',
              arguments=['0'],
              remappings=[('input/image',  f'slot{idx}/depth_image'),
                          ('output/image', f'slot{idx}/optical/depth_image'),
                         ]
          )

          payloads.append(rgbd_pointcloud_bridge)
          payloads.append(image_optical_frame)
          payloads.append(depth_image_optical_frame)
      else:
          print('Unknown payload: ', payload)

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
      parameters=[
          {"world_frame": world_name}
      ]
  )

  group_action = GroupAction([
        PushRosNamespace(model_name),
        ros2_ign_imu_bridge,
        ros2_ign_magnetometer_bridge,
        ros2_ign_air_pressure_bridge,
        ros2_ign_twist_bridge,
        ros2_ign_pose_bridge,
        ros2_ign_pose_static_bridge,
        ros2_tf_broadcaster,
        *payloads
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

  # thrust cmd
  left_thrust_topic = '/model/' + model_name + '/joint/left_engine_propeller_joint/cmd_thrust'
  right_thrust_topic = '/model/' + model_name + '/joint/right_engine_propeller_joint/cmd_thrust'
  ros2_ign_thrust_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=[left_thrust_topic + '@std_msgs/msg/Float64@ignition.msgs.Double',
                 right_thrust_topic + '@std_msgs/msg/Float64@ignition.msgs.Double'],
      remappings=[(left_thrust_topic, 'left/thrust/cmd_thrust'),
                  (right_thrust_topic, 'right/thrust/cmd_thrust')]
  )

  # thrust joint pos cmd
  # left_joint_topic = '/model/' + model_name + '/joint/left_chasis_engine_joint/0/cmd_pos'
  # right_joint_topic = '/model/' + model_name + '/joint/right_chasis_engine_joint/0/cmd_pos'
  left_joint_topic = '/usv/left/thruster/joint/cmd_pos'
  right_joint_topic = '/usv/right/thruster/joint/cmd_pos'

  ros2_ign_thrust_joint_bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      output='screen',
      arguments=[left_joint_topic + '@std_msgs/msg/Float64@ignition.msgs.Double',
                 right_joint_topic + '@std_msgs/msg/Float64@ignition.msgs.Double'],
      remappings=[(left_joint_topic, 'left/thrust/joint/cmd_pos'),
                  (right_joint_topic, 'right/thrust/joint/cmd_pos')]
  )

  group_action = GroupAction([
        PushRosNamespace(model_name),
        ros2_ign_thrust_bridge,
        ros2_ign_thrust_joint_bridge,
  ])

  handler = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=ignition_spawn_entity,
          on_exit=[group_action],
      ))

  return [ignition_spawn_entity, handler]


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

        DeclareLaunchArgument(
            'flightTime',
            default_value='10',
            description='Battery flight time in minutes (only for UAVs)'),

        DeclareLaunchArgument(
            'slot0',
            default_value='',
            description='Payload mounted to slot 0'),
        DeclareLaunchArgument(
            'slot0_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot1',
            default_value='',
            description='Payload mounted to slot 1'),
        DeclareLaunchArgument(
            'slot1_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot2',
            default_value='',
            description='Payload mounted to slot 2'),
        DeclareLaunchArgument(
            'slot2_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot3',
            default_value='',
            description='Payload mounted to slot 3'),
        DeclareLaunchArgument(
            'slot3_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        # launch setup
        OpaqueFunction(function = launch)
    ])
