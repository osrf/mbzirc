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

#include <ignition/msgs/param_v.pb.h>

#include <string>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "RFRange.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::RFRangePrivate
{
  /// \brief The link entity.
  public: ignition::gazebo::Link link;

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Topic to publish the sensor output.
  public: std::string topic = "/sensor/rfrange";

  /// \brief An Ignition Transport node for communications.
  public: ignition::transport::Node node;

  /// \brief An Ignition Transport publisher.
  public: ignition::transport::Node::Publisher pub;

  /// \brief Last system update simulation time.
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief System update period calculated from <update_rate>.
  public: std::chrono::steady_clock::duration updatePeriod{0};
};


//////////////////////////////////////////////////
RFRange::RFRange()
  : dataPtr(std::make_unique<RFRangePrivate>())
{
  ignerr << "RFRange initialized" << std::endl;
}

//////////////////////////////////////////////////
void RFRange::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Initialize system update period
  double rate = _sdf->Get<double>("update_rate", 1).first;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->dataPtr->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  // // Parse required elements.
  // if (!_sdf->HasElement("link_name"))
  // {
  //   ignerr << "No <link_name> specified" << std::endl;
  //   return;
  // }

  // std::string linkName = _sdf->Get<std::string>("link_name");
  // this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
  // if (!this->dataPtr->link.Valid(_ecm))
  // {
  //   ignerr << "Could not find link named [" << linkName
  //          << "] in model" << std::endl;
  //   return;
  // }

  this->dataPtr->pub =
    this->dataPtr->node.Advertise<msgs::Param_V>(this->dataPtr->topic);
}

//////////////////////////////////////////////////
void RFRange::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("RFRange::PreUpdate");

  if (_info.paused)
    return;

  // Throttle update rate
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->updatePeriod)
  {
    return;
  }
  this->dataPtr->lastUpdateTime = _info.simTime;

  // Publish output.
  ignition::msgs::Param_V outputMsg;
  auto *param = outputMsg.add_param()->mutable_params();

  ignition::msgs::Any modelValue;
  modelValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_STRING);
  modelValue.set_string_value("model1");

  // Set the model field.
  (*param)["model"] = modelValue;

  ignition::msgs::Any rangeValue;
  rangeValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  rangeValue.set_double_value(2.5);

  // Set the playback field.
  (*param)["range"] = rangeValue;

  this->dataPtr->pub.Publish(outputMsg);
}

IGNITION_ADD_PLUGIN(RFRange,
                    ignition::gazebo::System,
                    RFRange::ISystemConfigure,
                    RFRange::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFRange,
                          "ignition::gazebo::systems::RFRange")
