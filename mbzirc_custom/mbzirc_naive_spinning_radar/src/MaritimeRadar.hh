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

#include <memory>

#include <sdf/sdf.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/float_v.pb.h>

namespace mbzirc
{
  /// \brief An example class to simulate a radar that generates range
  /// and bearing data. Essentially it takes a LiDAR and projects it
  /// onto a polar plot. The range noise of the lidar should be tuned to be
  /// within the limits of the radar's noise uncertainty.
  ///
  /// # Parameters
  /// * laser_topic - The topic of the associated LiDAR.
  /// * angular_resolution - The angular resolution of each spoke. Normally
  ///   marine radars publish their position and a sample image of returns along
  ///   a given "spoke".
  /// * linear_resolution - The linear resolution of each range bin. Marine
  /// radars usually return a 1D array for range bins for a given angle.
  /// * max_range - The maximum range of the radar.
  /// * min_range - The minimum range of the radar.
  /// * debug_image_topic - The topic to publish the debug image. This is a
  ///   ignition::msgs::Image message with the full polarplot and scan line,
  ///   Useful for debugging.
  /// * radar_spoke_topic - The topic to publish the radar spoke. This is a
  ///   ignition::msgs::Float_V message with the range bins for a given angle.
  ///   The first number in this float_v message is the angle of the spoke.
  ///   The second number is the angular resolution of the spoke. The third
  ///   number is the linear resolution of the spoke. The rest of the numbers
  ///   are the range bins for the spoke with measurements in DB.
  /// By default this system has been tuned to us the Wartsila RS24 radar
  /// parameters.
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
    /// This turns the lidar sensor and plots the range data as individual bins.
    public: void OnRadarScan(const ignition::msgs::LaserScan &_msg);

    /// \brief Publishes a RADAR scan message. This function
    public: void PublishScan();

    /// \brief clear the current scan buffer
    /// \param[in] index - Spoke to be cleared.
    public: void ClearScanBuffer(std::size_t index);

    /// \brief Minimum range (m)
    public: double minRange{1.0};

    /// \brief Maximum range (m)
    public: double maxRange{1500.0};

    /// \brief Resolution (in metres)
    public: double resolution{0.75};

    /// \brief Minimum vertical angle (rad)
    public: double minVerticalAngle{-0.1745};

    /// \brief Maximum vertical angle (rad)
    public: double maxVerticalAngle{0.1745};

    /// \brief Angular resolution (rad)
    public: double angularResolution{IGN_DTOR(1.44)};

    /// \brief Sensor update rate
    public: double updateRate{1.0};

    /// \brief Number of beams of the sampling GPURay
    public: size_t numBeams{0};

    /// \brief Joint Position
    public: std::size_t radarBinIndex;

    /// \brief stores radar scan data for each beam.
    public: std::vector<std::vector<double>> radarBin;

    /// \brief Laser scan topic name
    public: std::string laserTopic{"scan"};

    /// \brief Laser scan initialized
    public: bool laserInitialized{false};

    /// \brief Noise to be applied to radar data
    ///public: ignition::sensors::NoisePtr noise;

    /// \brief Entity ID of the sensor
    public: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};

    /// \brief Entity ID of the rotary joint.
    public: ignition::gazebo::Entity jointEntity{ignition::gazebo::kNullEntity};

    /// \brief Ignition tranport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport publisher for publishing sensor data
    public: ignition::transport::Node::Publisher linePub;

    /// \brief Ignition transport publisher for publishing sensor data
    public: ignition::transport::Node::Publisher debugPub;

    /// \brief Image used for debugging radar scan.
    public: ignition::msgs::Image image;

    public: ignition::msgs::Float_V lineBin;

    public: const size_t debugImageWidth{640};

    public: std::string debugImageTopic{"/radar/debug_image"};

    public: std::string radarSpokeTopic{"/radar/scan"};

    public: std::mutex mtx;
  };
}

#endif
