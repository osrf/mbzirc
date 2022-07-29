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

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Image.hh>

#include "BaseStation.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace mbzirc;

/////////////////////////////////////////////////
BaseStation::BaseStation()
{
}

/////////////////////////////////////////////////
BaseStation::~BaseStation() = default;

/////////////////////////////////////////////////
void BaseStation::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  std::string defaultVideoTopic{"/base_station/video"};
  auto videoTopic = _sdf->Get<std::string>("video_topic",
      defaultVideoTopic).first;
  this->node.Subscribe(
    videoTopic,
    &BaseStation::OnVideo,
    this);

  std::string defaultTargetTopic{"/base_station/target"};
  auto targetTopic = _sdf->Get<std::string>("target_topic",
      defaultTargetTopic).first;
  this->node.Subscribe(
    targetTopic,
    &BaseStation::OnTarget,
    this);
}

//////////////////////////////////////////////////
void BaseStation::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->simTime = _info.simTime;
}


//////////////////////////////////////////////////
void BaseStation::OnVideo(const ignition::msgs::Dataframe &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  ignition::msgs::StringMsg msg;
  msg.ParseFromString(_msg.data());

  if (this->sensorFrame.empty() || this->sensorFrame != msg.data())
  {
    this->prevVideoSimTime = this->simTime;
    this->simTimes.clear();
    this->sensorFrame = msg.data();

    std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
        [&](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
    {
      if (!_result)
      {
        ignerr << "Error sending stream start request" << std::endl;
        this->sensorFrame.clear();
       }
    };

    ignition::msgs::StringMsg_V req;
    req.add_data(this->sensorFrame);
    bool result = this->node.Request("/mbzirc/target/stream/start", req, cb);
    if (!result)
      this->sensorFrame.clear();
    return;
  }

  auto t = this->simTime - this->prevVideoSimTime;
  double sec = std::chrono::duration_cast<std::chrono::milliseconds>(t).count()
      * 1e-3;
  if (math::equal(sec, 0.0))
    return;

  this->prevVideoSimTime = this->simTime;
  this->simTimes.push_back(t);
  this->simTimeSum += sec;

  if (this->simTimes.size() >= this->kVideoStreamWindowSize)
  {
    double avgT = this->simTimeSum / this->kVideoStreamWindowSize;
    if ((1 / avgT) < this->kminStreamRate)
    {
      // std::cerr << "video stream is lower than "
      //           << std::to_string(this->kminStreamRate)
      //           << " Hz" << std::endl;
    }

    auto front = this->simTimes.front();
    double fsec = std::chrono::duration_cast<std::chrono::milliseconds>(
        front).count() * 1e-3;
    this->simTimeSum -= fsec;
    this->simTimes.pop_front();
  }
}

//////////////////////////////////////////////////
void BaseStation::OnTarget(const ignition::msgs::Dataframe &_msg)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending target report" << std::endl;
  };

  auto data = _msg.data();
  ignition::msgs::StringMsg_V req;
  req.ParseFromString(data);
  bool result = this->node.Request("/mbzirc/target/stream/report", req, cb);
}

IGNITION_ADD_PLUGIN(mbzirc::BaseStation,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPostUpdate)

