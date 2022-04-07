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
#ifndef MBZIRC_IGN_BASESTATION_HH_
#define MBZIRC_IGN_BASESTATION_HH_

#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs/dataframe.pb.h>

namespace mbzirc
{
  /// \brief A plugin that validates target identification reports.
  class BaseStation:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: BaseStation();

    /// \brief Destructor
    public: ~BaseStation();

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    // public: void OnVideoStart(const ignition::msgs::Dataframe &_msg);
    public: void OnVideo(const ignition::msgs::Dataframe &_msg);
    public: void OnTarget(const ignition::msgs::Dataframe &_msg);

    /// \brief Ignition transport node
    public: ignition::transport::Node node;
    public: ignition::transport::Node::Publisher pub;

    public: std::mutex mutex;
    /// \brief Current simulation time.
    public: std::chrono::steady_clock::duration simTime;
    public: std::chrono::steady_clock::duration prevVideoSimTime;
    public: std::chrono::steady_clock::duration simTimeSum;
    public: std::list<std::chrono::steady_clock::duration> simTimes;

    public: const unsigned int kVideoStreamWindowSize = 10u;
    public: const unsigned int kminStreamRate = 10u;
    // public: bool newStream = false;
    public: std::string sensorFrame;
  };
}

#endif
