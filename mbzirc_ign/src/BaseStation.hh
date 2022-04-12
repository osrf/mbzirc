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

    /// \brief Callback when video stream is received
    /// \param[in] _msg Dataframe message containing info on the
    /// sensor frame associated with the video stream
    public: void OnVideo(const ignition::msgs::Dataframe &_msg);

    /// \brief Callback when target is reported
    /// \param[in] _msg Dataframe message containing info on the
    /// targets and their image position
    public: void OnTarget(const ignition::msgs::Dataframe &_msg);

    /// \brief Ignition transport node
    private: ignition::transport::Node node;

    /// \brief Ignition transport publisher
    private: ignition::transport::Node::Publisher pub;

    /// \brief mutex to protect sim time
    private: std::mutex mutex;

    /// \brief Current simulation time.
    private: std::chrono::steady_clock::duration simTime;

    /// \brief Previous sim time when video is received
    private: std::chrono::steady_clock::duration prevVideoSimTime;

    /// \brief Sum of sim time over a window of size kVideoStreamWindowSize
    private: double simTimeSum = 0.0;

    /// \brief A list of sim times. List size = kVideoStreamWindowSize
    private: std::list<std::chrono::steady_clock::duration> simTimes;

    /// \brief Window of sim times used to compute average video stream rate
    private: const unsigned int kVideoStreamWindowSize = 10u;

    /// \brief Min allowed video stream rate
    private: const unsigned int kminStreamRate = 10u;

    /// \brief Frame of sensor associated with the incoming video stream
    public: std::string sensorFrame;
  };
}

#endif
