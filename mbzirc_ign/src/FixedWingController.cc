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
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include <mavros_msgs/msg/nav_controller_output.hpp>

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
  double targetVelocity{0};

public:
  Entity entity;

public:
  math::Vector3d pitchAxis;

public:
  math::Vector3d rollAxis;

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

  /// Get the link that the controller should track
  if (_sdf->HasElement("link_name"))
  {
    auto linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->entity = model.LinkByName(_ecm, "wing");

    if (this->dataPtr->entity == kNullEntity)
    {
      ignerr << "Could not find link named " << linkName << std::endl;
      return;
    }
  }
  else
  {
    ignerr << "Please specify a link name" << std::endl;
    return;
  }

  enableComponent<components::WorldPose>(_ecm, this->dataPtr->entity);
  enableComponent<components::WorldLinearVelocity>(_ecm, this->dataPtr->entity);

  if (_sdf->HasElement("left_flap"))
  {
    auto leftFlapTopic = _sdf->Get<std::string>("left_flap");
    this->dataPtr->leftFlap =
      this->dataPtr->node.Advertise<msgs::Double>(leftFlapTopic);
  }

  if (_sdf->HasElement("right_flap"))
  {
    auto rightFlapTopic = _sdf->Get<std::string>("right_flap");
    this->dataPtr->rightFlap =
      this->dataPtr->node.Advertise<msgs::Double>(rightFlapTopic);
  }

  if (_sdf->HasElement("propeller"))
  {
    auto propellerTopic = _sdf->Get<std::string>("propeller");
    this->dataPtr->thruster =
      this->dataPtr->node.Advertise<msgs::Double>(propellerTopic);
  }


  this->dataPtr->logFile.open("zephyr_controller.csv");

  if (_sdf->HasElement("roll_p"))
  {
    this->dataPtr->rollControl.SetPGain(_sdf->Get<double>("roll_p"));
  }
  if (_sdf->HasElement("roll_i"))
  {
    this->dataPtr->rollControl.SetIGain(_sdf->Get<double>("roll_i"));
  }
  if (_sdf->HasElement("roll_d"))
  {
    this->dataPtr->rollControl.SetDGain(_sdf->Get<double>("roll_d"));
  }

  if (_sdf->HasElement("pitch_p"))
  {
    this->dataPtr->pitchControl.SetPGain(_sdf->Get<double>("pitch_p"));
  }
  if (_sdf->HasElement("pitch_i"))
  {
    this->dataPtr->pitchControl.SetIGain(_sdf->Get<double>("pitch_i"));
  }
  if (_sdf->HasElement("pitch_d"))
  {
    this->dataPtr->pitchControl.SetDGain(_sdf->Get<double>("pitch_d"));
  }

  if (_sdf->HasElement("velocity_p"))
  {
    this->dataPtr->velocityControl.SetPGain(_sdf->Get<double>("velocity_p"));
  }
  if (_sdf->HasElement("velocity_i"))
  {
    this->dataPtr->velocityControl.SetIGain(_sdf->Get<double>("velocity_i"));
  }
  if (_sdf->HasElement("velocity_d"))
  {
    this->dataPtr->velocityControl.SetDGain(_sdf->Get<double>("velocity_d"));
  }

  if (_sdf->HasElement("pitch_axis"))
  {
    this->dataPtr->pitchAxis = _sdf->Get<ignition::math::Vector3d>("pitch_axis");
  }
  if (_sdf->HasElement("roll_axis"))
  {
    this->dataPtr->rollAxis = _sdf->Get<ignition::math::Vector3d>("roll_axis");
  }
}

void FixedWingControllerPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->entity);
  auto rotation = pose->Data().Rot().Euler();

  auto linVelocityComp =
    _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->entity);
  auto linVel = linVelocityComp->Data().Length();

  /// Fix Thruster power
  msgs::Double thrusterPower;
  thrusterPower.set_data(1000.0);
  this->dataPtr->thruster.Publish(thrusterPower);

  /// Attitude control
  auto rollError =
    rotation.Dot(this->dataPtr->rollAxis) - this->dataPtr->targetRoll;
  auto difference = this->dataPtr->rollControl.Update(rollError, _info.dt);

  auto pitchError =
    rotation.Dot(this->dataPtr->pitchAxis) - this->dataPtr->targetPitch;
  auto offset = this->dataPtr->pitchControl.Update(pitchError, _info.dt);

  /// Control aerelions
  msgs::Double leftFlap;
  leftFlap.set_data(-offset + difference/2);
  this->dataPtr->leftFlap.Publish(leftFlap);

  msgs::Double rightFlap;
  rightFlap.set_data(-offset - difference/2);
  this->dataPtr->rightFlap.Publish(rightFlap);

  /// Log data
  this->dataPtr->logFile << rollError << "," << pitchError << "," << linVel << "\n";
  this->dataPtr->logFile.flush();
}

IGNITION_ADD_PLUGIN(
    mbzirc::FixedWingControllerPlugin,
    ignition::gazebo::System,
    mbzirc::FixedWingControllerPlugin::ISystemConfigure,
    mbzirc::FixedWingControllerPlugin::ISystemPreUpdate)
