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
#ifndef MBZIRC_CUSTOMIZATIONS_MARITIMERADAR_HH_
#define MBZIRC_CUSTOMIZATIONS_MARITIMERADAR_HH_

#include <atomic>
#include <memory>

#include <sdf/sdf.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/float_v.pb.h>

namespace mbzirc
{
  /// \brief An example class to simulate a radar that generates range, azimuth 
  /// and elevation data. Essentially it takes a 3D LiDAR, filters the data for
  /// acceptable data points based on the specifications, and translates it into
  /// radar scans depending on where the laser scan is facing.
  ///
  /// # Parameters
  /// * laser_topic - The topic of the associated LiDAR.
  /// * radar_scan_topic - The topic to publish the radar output. This is an
  ///   ignition::msgs::Float_V message with repeating sets of range, azimuth,
  ///   and elevation values.
  /// By default this system has been tuned to use the Wartsila RS24 radar
  /// parameters but modified to have a longer range than the original.
  class MaritimeRadar:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPostUpdate,
        public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: MaritimeRadar();

    /// \brief Destructor
    public: ~MaritimeRadar() override;

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
    /// This translates the lidar sensor data into radar data.
    public: void OnRadarScan(const ignition::msgs::LaserScan &_msg);

    /// \brief Joint Position
    public: std::atomic<double> radarSpokeAngle;

    /// \brief Laser scan topic name
    public: std::string laserTopic{"scan"};

    /// \brief Laser scan initialized
    public: bool laserSubscribed{false};

    /// \brief Frame ID of this radar sensor
    public: std::string frameId{"radar"};

    /// \brief Entity ID of the sensor
    public: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};

    /// \brief Entity ID of the rotary joint
    public: ignition::gazebo::Entity jointEntity{ignition::gazebo::kNullEntity};

    /// \brief Ignition tranport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport publisher for publishing sensor data
    public: ignition::transport::Node::Publisher radarScanPub;

    /// \brief Radar scan topic name
    public: std::string radarScanTopic{"/radar/scan"};
  };
}

#endif
