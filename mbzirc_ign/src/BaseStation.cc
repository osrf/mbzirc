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

  //// for testing
  // std::string topic = "/broker/msgs";
  // this->pub = this->node.Advertise<ignition::msgs::Dataframe>(topic);

    std::cerr << "base station init " << std::endl;
}

//////////////////////////////////////////////////
void BaseStation::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->simTime = _info.simTime;
  }


  {
//    msgs::Dataframe msg;
//    msg.set_src_address("base_station");
//    msg.set_dst_address("base_station");
//
//    ignition::msgs::StringMsg_V videoMsg;
//    videoMsg.add_data("vehicle_name");
//    videoMsg.add_data("vehicle_sensor");
//    std::string data;
//    videoMsg.SerializeToString(&data);
//    msg.set_data(data);
//    this->pub.Publish(msg);

//    CommsData data;
//    data.sensor_slot = 3;
//    data.x = 640;
//    data.y = 480;
//    data.image_width = 1280;
//    data.image_height = 960;

//    std::string d = std::string(reinterpret_cast<const char *>(&data));
//    msg.set_data(d);
    // this->pub.Publish(msg);


//    const CommsData *rmsg = reinterpret_cast<const CommsData *>(msg.data().c_str());
//    std::cerr << "target recevied: " << std::endl;
//    std::cerr << "  " << rmsg->sensor_slot << std::endl;
//    std::cerr << "  " << rmsg->x << std::endl;
//    std::cerr << "  " << rmsg->y << std::endl;
//    std::cerr << "  " << rmsg->image_width << std::endl;
//    std::cerr << "  " << rmsg->image_height << std::endl;
  }

}

//////////////////////////////////////////////////
// void BaseStation::OnVideoStart(const ignition::msgs::Dataframe &_msg)
// {
//   auto data = _msg.data();
//   ignition::msgs::StringMsg_V msg;
//   msg.ParseFromString(data);
//   std::cerr << "received video start request " << std::endl;
//   std::cerr << "  vehicle: " << msg.data(0);
//   std::cerr << "  sensor: " << msg.data(1);
//   bool result = this->node.Request("/mbzirc/target/stream/start", msg);
//   std::cerr << "result " << result << std::endl;
//   this->newStream = true;
// }

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

    std::cerr << "received video, sensor frame " << msg.data() << std::endl;

    ignition::msgs::StringMsg_V msg;
    msg.add_data(this->sensorFrame);
    bool result = this->node.Request("/mbzirc/target/stream/start", msg);
    std::cerr << "starting stream " << result << std::endl;
    return;
  }

  auto t = this->simTime - this->prevVideoSimTime;
  this->simTimes.push_back(t);
  this->simTimeSum += t;

  if (this->simTimes.size() >= this->kVideoStreamWindowSize)
  {
    int64_t s, ns;
    std::tie(s, ns) = ignition::math::durationToSecNsec(this->simTimeSum);
    double t = s + static_cast<double>(ns) / 1e9;
    t = t / this->kVideoStreamWindowSize;
    std::cerr << " t " << t << std::endl;
    if (1 / t < this->kminStreamRate)
    {
      std::cerr << "video stream is lower than " << std::to_string(this->kminStreamRate)
                << " Hz" << std::endl;
    }

    auto front = this->simTimes.front();
    this->simTimeSum - front;
    this->simTimes.pop_front();
  }
}

//////////////////////////////////////////////////
void BaseStation::OnTarget(const ignition::msgs::Dataframe &_msg)
{
  auto data = _msg.data();
  ignition::msgs::StringMsg_V msg;
  msg.ParseFromString(data);
  std::cerr << "received target report " << std::endl;
  std::cerr << "  type: " << msg.data(0);
  std::cerr << "  x: " << msg.data(1);
  std::cerr << "  y: " << msg.data(2);

  bool result = this->node.Request("/mbzirc/target/stream/report", msg);
  std::cerr << "result " << result << std::endl;
}

IGNITION_ADD_PLUGIN(mbzirc::BaseStation,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPostUpdate)

