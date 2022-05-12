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
#ifndef MBZIRC_CUSTOMIZATIONS_NAIVERADAR_HH_
#define MBZIRC_CUSTOMIZATIONS_NAIVERADAR_HH_

#include <memory>

#include <sdf/sdf.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace mbzirc
{
  /// \brief A example class to simulate a radar that generates range
  /// and bearing data
  class NaiveRadar:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: NaiveRadar();

    /// \brief Destructor
    public: ~NaiveRadar() override;

    // Documentation inherited.
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Minimum range (m)
    public: double minRange{1.0};

    /// \brief Maximum range (m)
    public: double maxRange{1000.0};

    /// \brief Minimum angle (rad)
    public: double minAngle{-IGN_PI};

    /// \brief Maximum angle (rad)
    public: double maxAngle{IGN_PI};

    /// \brief Minimum vertical angle (rad)
    public: double minVerticalAngle{-0.1745};

    /// \brief Maximum vertical angle (rad)
    public: double maxVerticalAngle{0.1745};

    /// \brief Sensor update rate
    public: double updateRate{1.0};

    /// \brief Noise to be applied to radar data
    public: ignition::sensors::NoisePtr noise;

    /// \brief Entity ID of the sensor
    public: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};

    /// \brief Entity ID of the parent model
    public: ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};

    /// \brief Ignition tranport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport publisher for publishing sensor data
    public: ignition::transport::Node::Publisher publisher;

    /// \brief Sim time when next update should occur
    public: std::chrono::steady_clock::duration nextUpdateTime
        {std::chrono::steady_clock::duration::zero()};
  };
}

#endif
