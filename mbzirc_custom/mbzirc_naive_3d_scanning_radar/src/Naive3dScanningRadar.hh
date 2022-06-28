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
#ifndef MBZIRC_CUSTOMIZATIONS_NAIVE3DSCANNINGRADAR_HH_
#define MBZIRC_CUSTOMIZATIONS_NAIVE3DSCANNINGRADAR_HH_

#include <memory>

#include <sdf/sdf.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/float_v.pb.h>

namespace mbzirc
{
  /// \brief An example class to simulate a radar that generates range, azimuth,
  /// and elevation data. Essentially it takes a 3D LiDAR and filters the data
  /// for acceptable data points based on the specifications.
  ///
  /// # Parameters
  /// * laser_topic - The topic of the associated LiDAR.
  /// * radar_scan_topic - The topic to publish the radar output. This is an
  ///   ignition::msgs::Float_V message with repeating sets of range, azimuth,
  ///   and elevation values.
  /// By default this system has been tuned to use the Wartsila RS24 radar
  /// parameters but modified to have a longer range than the original.
  class Naive3dScanningRadar:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPostUpdate,
        public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: Naive3dScanningRadar();

    /// \brief Destructor
    public: ~Naive3dScanningRadar() override;

    // Documentation inherited.
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Callback for laser scan messages
    public: void OnRadarScan(const ignition::msgs::LaserScan &_msg);

    /// \brief Laser scan topic name
    public: std::string laserTopic{"scan"};

    /// \brief Laser scan subscribed
    public: bool laserSubscribed{false};

    /// \brief Radar scan topic name
    public: std::string radarScanTopic{"/radar/scan"};

    /// \brief Frame ID of this radar sensor
    public: std::string frameId{"radar"};

    /// \brief Entity ID of the sensor
    public: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};

    /// \brief Ignition tranport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport publisher for publishing radar data
    public: ignition::transport::Node::Publisher radarScanPub;
  };
}

#endif
