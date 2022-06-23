/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef MBZIRC_IGN__COMPONENTS_HH_
#define MBZIRC_IGN__COMPONENTS_HH_

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>

namespace mbzirc
{
/// \brief Structure to hold information about competitor platform sensors
struct SensorInfo
{
  /// \brief Entity that the sensor is attached to
  ignition::gazebo::Entity sensorEntity;

  /// \brief Name of the platform slot the sensor is attached to
  std::string slotName;

  /// \brief Sensor type in (camera, rgbd_camera, gpu_lidar)
  std::string sensorType;
};

/// \brief Structure to hold information about competitor platforms
struct PlatformInfo
{
  /// \brief Entity of tAhe platform
  ignition::gazebo::Entity modelEntity;

  /// \brief Name of the platform
  std::string robotName;

  /// \brief Spawn position of the platform
  ignition::math::Vector3d initialPos;

  /// \brief Sensors attached to the platform
  std::vector<SensorInfo> sensors;
};

namespace components
{
  /// \brief A component that identifies an entity as being a spawned platform.
  using CompetitorPlatform = ignition::gazebo::components::Component<PlatformInfo, class CompetitorPlatformTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("mbzirc_components.CompetitorPlatform", CompetitorPlatform)

}  // namespace components
}  // namespace mbzirc

#endif  // MBZIRC_IGN__COMPONENTS_HH_

