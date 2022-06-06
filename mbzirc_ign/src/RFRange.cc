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

#include <memory>
#include <string>
#include <unordered_map>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "RFRange.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
RFRangeSensor::RFRangeSensor()
{
  ignerr << "RFRangeSensor constructor" << std::endl;
}

//////////////////////////////////////////////////
RFRangeSensor::~RFRangeSensor() = default;

//////////////////////////////////////////////////
void RFRangeSensor::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto entity = _ecm.CreateEntity();

  _ecm.SetParentEntity(entity, _entity);
  _ecm.CreateComponent(entity, RFRangeType());

  ignition::math::Pose3d pose;
  if (_ecm.EntityHasComponentType(_entity, gazebo::components::Model::typeId))
  {
    auto modelPose = _ecm.Component<gazebo::components::Pose>(_entity);
    pose += modelPose->Data();
    _ecm.CreateComponent(entity, gazebo::components::WorldPose(pose));
  }
}

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

  /// \brief ToDO.
  public: std::vector<ignition::gazebo::Entity> entityVector;
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

  _ecm.EachNew<RFRangeType,
               ignition::gazebo::components::WorldPose>(
    [&](const ignition::gazebo::Entity &_entity,
        const RFRangeType *_custom,
        const ignition::gazebo::components::WorldPose *_pose)->bool
      {
        auto parentId = ignition::gazebo::topLevelModel(_entity, _ecm);

        ignerr << "Parent: " << _ecm.ParentEntity(_entity) << std::endl;

        // Keep track of this sensor.
        this->dataPtr->entityVector.push_back(_ecm.ParentEntity(_entity));

        ignerr << "New sensor detected at " << _pose->Data() << std::endl;

        return true;
      });

  if (_info.paused)
    return;

  // Throttle update rate.
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->updatePeriod)
  {
    return;
  }
  this->dataPtr->lastUpdateTime = _info.simTime;

  for (auto entityId : this->dataPtr->entityVector)
  {
    ignerr << "Sensor at [" << ignition::gazebo::worldPose(entityId, _ecm)
           << "]" << std::endl;
  }

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

IGNITION_ADD_PLUGIN(RFRangeSensor,
                    ignition::gazebo::System,
                    RFRangeSensor::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(RFRangeSensor,
                          "ignition::gazebo::systems::RFRangeSensor")
