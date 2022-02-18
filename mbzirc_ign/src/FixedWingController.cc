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
#include "FixedWingController.hh"

#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

/// \brief fstream for temporary file storage logging
#include <fstream>

using namespace mbzirc;
using namespace ignition;
using namespace gazebo;

/// This file describes a simple Attitude controller for fixed wing air craft such as the
/// Zephyr. The controller is based on three PID loops: one to control the pitch
/// rate, one to control the roll rate, and one to control the forward velocity.
/// This yaw rate in the zephyr is affected by the roll rate as the zephyr
/// itself is underactuated.
class mbzirc::FixedWingControllerPrivate
{
public:
  math::PID rollControl;

public:
  math::PID pitchControl;

public:
  math::PID velocityControl;

public:
  double targetRoll{0};

public:
  double targetPitch{0};

public:
  Entity entity;

public:
  transport::Node node;

public:
  transport::Node::Publisher leftFlap;

public:
  transport::Node::Publisher rightFlap;

public:
  transport::Node::Publisher thruster;

public:
  std::ofstream logFile;
};

FixedWingControllerPlugin::FixedWingControllerPlugin() : dataPtr(std::make_unique<FixedWingControllerPrivate>())
{
}

FixedWingControllerPlugin::~FixedWingControllerPlugin()
{
}

void FixedWingControllerPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  auto model = Model(_entity);

  this->dataPtr->entity = model.LinkByName(_ecm, "wing");
  ;
  enableComponent<components::WorldPose>(_ecm, this->dataPtr->entity);
  enableComponent<components::WorldLinearVelocity>(_ecm, this->dataPtr->entity);
  enableComponent<components::WorldAngularVelocity>(_ecm, this->dataPtr->entity);
  enableComponent<components::LinearVelocity>(_ecm, this->dataPtr->entity);
  enableComponent<components::AngularVelocity>(_ecm, this->dataPtr->entity);

  this->dataPtr->leftFlap = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/zephyr/joint/left_flap_joint/cmd_pos");
  this->dataPtr->rightFlap = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/zephyr/joint/right_flap_joint/cmd_pos");
  this->dataPtr->thruster = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/zephyr/joint/propeller_joint/cmd_vel");

  this->dataPtr->logFile.open("zephyr_controller.log");

  this->dataPtr->rollControl.SetPGain(0.1);
  this->dataPtr->pitchControl.SetPGain(1);
}

void FixedWingControllerPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->entity);
  auto rotation = pose->Data().Rot().Euler();

  auto angularVelocity =
      _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->entity);
  auto linearVelocity =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->entity);

  /// Fix Thruster power
  msgs::Double thrusterPower;
  thrusterPower.set_data(1000.0);
  this->dataPtr->thruster.Publish(thrusterPower);

  /// Attitude control
  auto difference = this->dataPtr->rollControl.Update(
      rotation.Y() - this->dataPtr->targetRoll, _info.dt);
  auto offset = this->dataPtr->pitchControl.Update(
      rotation.X() - this->dataPtr->targetPitch, _info.dt);

  /// Control aerelions
  msgs::Double leftFlap;
  leftFlap.set_data(-offset + difference/2);
  this->dataPtr->leftFlap.Publish(leftFlap);

  msgs::Double rightFlap;
  rightFlap.set_data(-offset - difference/2);
  this->dataPtr->rightFlap.Publish(rightFlap);


  /// Log data
  this->dataPtr->logFile
      <<  << "\n";
  this->dataPtr->logFile.flush();
}

IGNITION_ADD_PLUGIN(
    mbzirc::FixedWingControllerPlugin,
    ignition::gazebo::System,
    mbzirc::FixedWingControllerPlugin::ISystemConfigure,
    mbzirc::FixedWingControllerPlugin::ISystemPreUpdate)
