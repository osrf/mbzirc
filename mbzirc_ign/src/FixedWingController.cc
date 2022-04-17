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

#include <condition_variable>

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
/// Zephyr. The controller is based on two PID loops: one to control the pitch,
/// one to control the roll, and open loop forward velocity.
/// This yaw rate in the zephyr is affected by the roll rate as the zephyr
/// itself is underactuated.
class mbzirc::FixedWingControllerPrivate
{
  /// \brief Modes
  public: enum class Mode
  {
    /// \brief IDLE mode
    IDLE,
    /// \brief Take off mode
    TAKE_OFF_START,
    /// \brief Enter into a climb
    TAKE_OFF_CLIMB,
    /// \brief Just use to maintain attitude
    ATTITUDE_CONTROL
  };

  /// \brief Controller mode
  public: Mode mode{Mode::IDLE};

  /// \brief Roll control PID
  public: math::PID rollControl;

  /// \brief Pitch control PID
  public: math::PID pitchControl;

  /// \brief Target Roll control PID
  public: double targetRoll{0};

  /// \brief Target Pitch control setpoint
  public: double targetPitch{0};

  /// \brief Target velocity control setpoint
  public: double targetVelocity{0};

  /// \brief  Power required for take-off
  public: double takeOffPower{1000};

  /// \brief Take off target speed
  public: double takeOffSpeed{20};

  /// \brief Take off target altitude
  public: double takeOffAltitude{40};

  /// \brief Take off min pitch
  public: double takeOffMinPitch{0.1};

  /// \brief Entity to be controlled
  public: Entity entity;

  /// \brief Pitch Axis
  public: math::Vector3d pitchAxis;

  /// \brief Roll Axis
  public: math::Vector3d rollAxis;

  /// \brief Ignition Node for joint position controllers
  public: transport::Node node;

  /// \brief Ignition Publisher for left flap position controllers
  public: transport::Node::Publisher leftFlap;

  /// \brief Ignition Node for right flap position controllers
  public: transport::Node::Publisher rightFlap;

  /// \brief Ignition Node for thruster control
  public: transport::Node::Publisher thruster;

  /// \brief for synchronizing with takeoff
  public: std::condition_variable takeOffCv;

  /// \brief for synchronizing with takeoff
  public: std::mutex takeOffMutex;

  /// \brief for synchronizing with takeoff
  public: bool takeOffFlag{false};

  /// Uncomment to enable logging for PID tuning
  ///public: std::ofstream logFile;

  /// \brief Mutex for setpoints
  public: std::mutex mutex;

  /// \brief Setup the ros node
  /// \param[in] _name Ros node name
  public: void SetupSubscribers(std::string _name)
  {
    node.Subscribe(_name+"/cmd/target_attitude",
      &FixedWingControllerPrivate::OnAttitudeTarget, this);

    node.Advertise(_name+"/cmd/takeoff",
      &FixedWingControllerPrivate::TakeOffService, this);
  }



  /// \brief Callback for attitude target messages
  /// \param[in] _msg Attitude target message
  public: void OnAttitudeTarget(const msgs::Float_V& _msg)
  {
    if (_msg.data_size() != 5)
      ignerr << "Malformed attitude target message" << std::endl;
    /// Message format is as follows:
    /// [q,x,y,z,thrust]
    math::Quaterniond quat(
      _msg.data(0),
      _msg.data(1),
      _msg.data(2),
      _msg.data(3));

    std::lock_guard<std::mutex> lock(this->mutex);
    math::Vector3d euler = quat.Euler();
    targetRoll = euler.X();
    targetPitch = euler.Y();
    targetVelocity = _msg.data(4);

    this->pitchControl.Reset();
    this->rollControl.Reset();
  }

  /// \brief Take off service
  /// \param[in] takeOffParams - Take off parameters. A float array of 2
  /// elements. First being pitch, second being target altitude.
  /// \param[out] _result - Result of the service call.
  public: bool TakeOffService(
    const msgs::Float_V& _takeoffParams,
    msgs::Boolean &_result)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (_takeoffParams.data_size() != 2)
    {
      ignerr << "Malformed takeoff parameters" << std::endl;
      _result.set_data(false);
      return false;
    }
    this->takeOffMinPitch = _takeoffParams.data(0);
    this->takeOffAltitude = _takeoffParams.data(1);
    this->mode = Mode::TAKE_OFF_START;
    _result.set_data(true);

    this->pitchControl.Reset();
    this->rollControl.Reset();
    return true;
  }

  /// \brief Execute the control loop
  /// \param[in] _rotation -The attitude which you would like to maintain.
  public: void ExecuteAttitudeControlLoop(
    const math::Vector3d &_rotation,
    const UpdateInfo &_info)
  {
    /// Fix Thruster power
    msgs::Double thrusterPower;
    thrusterPower.set_data(this->targetVelocity);
    this->thruster.Publish(thrusterPower);

    /// Attitude control
    auto rollError =
      _rotation.Dot(this->rollAxis) - this->targetRoll;
    auto difference = this->rollControl.Update(rollError, _info.dt);

    auto pitchError =
      _rotation.Dot(this->pitchAxis) - this->targetPitch;
    auto offset = this->pitchControl.Update(pitchError, _info.dt);

    /// Control aerelions
    msgs::Double leftFlap;
    leftFlap.set_data(offset + difference/2);
    this->leftFlap.Publish(leftFlap);

    msgs::Double rightFlap;
    rightFlap.set_data(offset - difference/2);
    this->rightFlap.Publish(rightFlap);
  }
};

/////////////////////////////////////////////////////////////////////
FixedWingControllerPlugin::FixedWingControllerPlugin() :
  dataPtr(std::make_unique<FixedWingControllerPrivate>())
{
}

/////////////////////////////////////////////////////////////////////
FixedWingControllerPlugin::~FixedWingControllerPlugin() = default;

/////////////////////////////////////////////////////////////////////
void FixedWingControllerPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  char const** argv = NULL;
  auto model = Model(_entity);

  /// Get the link that the controller should track
  if (_sdf->HasElement("link_name"))
  {
    auto linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->entity = model.LinkByName(_ecm, linkName);

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

  /// Enable the Pose and Velocity components
  enableComponent<components::WorldPose>(_ecm, this->dataPtr->entity);
  enableComponent<components::WorldLinearVelocity>(_ecm, this->dataPtr->entity);
  enableComponent<components::WorldAngularVelocity>(_ecm, this->dataPtr->entity);

  /// Get Control surface topics
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

  /// Uncomment for logging
  //igndbg << "initiallized log file" << std::endl;
  //this->dataPtr->logFile.open("zephyr_take_off_controller.csv");

  /// Setup Roll PID
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

  /// Setup Pitch PID
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

  /// Get Axis in relation to link
  if (_sdf->HasElement("pitch_axis"))
  {
    this->dataPtr->pitchAxis = _sdf->Get<ignition::math::Vector3d>("pitch_axis");
  }
  if (_sdf->HasElement("roll_axis"))
  {
    this->dataPtr->rollAxis = _sdf->Get<ignition::math::Vector3d>("roll_axis");
  }

  // Take off power
  if (_sdf->HasElement("take_off_power"))
  {
    this->dataPtr->takeOffPower = _sdf->Get<double>("take_off_power");
  }

  // Minimum speed before we pivot to a climb
  if (_sdf->HasElement("take_off_speed"))
  {
    this->dataPtr->takeOffSpeed = _sdf->Get<double>("take_off_speed");
  }

  /// Start listening to attitude target messages
  this->dataPtr->SetupSubscribers(_sdf->Get<std::string>("model_name"));
}

/////////////////////////////////////////////////////////////////////
void FixedWingControllerPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  /// State machine for flight controller
  if (this->dataPtr->mode == FixedWingControllerPrivate::Mode::IDLE)
  {
    return;
  }

  auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->entity);
  auto rotation = pose->Data().Rot().Euler();

  auto cmd = 0.0;

  /// State machine for flight controller
  if (this->dataPtr->mode == FixedWingControllerPrivate::Mode::TAKE_OFF_START)
  {
    /// Goal of takeoff controller is to increase altitude as much as possible.
    /// Once a basic airspeed is achieved we adjust the angle of attack.
    /// Fix Thruster power-
    msgs::Double thrusterPower;
    thrusterPower.set_data(this->dataPtr->takeOffPower);
    this->dataPtr->thruster.Publish(thrusterPower);

    /// Keep the Aircraft on the runway till ready to take off
    auto pitchError = rotation.Dot(this->dataPtr->pitchAxis);
    auto offset = this->dataPtr->pitchControl.Update(pitchError, _info.dt);

    /// Control aerelions
    msgs::Double leftFlap;
    leftFlap.set_data(offset);
    this->dataPtr->leftFlap.Publish(leftFlap);

    msgs::Double rightFlap;
    rightFlap.set_data(offset);
    this->dataPtr->rightFlap.Publish(rightFlap);
    cmd = offset;

    auto linVelocityComp =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->entity);
    auto linVel = linVelocityComp->Data().Length();

    /// Detect if sufficient lift is achieved
    if (linVel > this->dataPtr->takeOffSpeed) {
      /// Transition into climb
      this->dataPtr->mode = FixedWingControllerPrivate::Mode::TAKE_OFF_CLIMB;
      this->dataPtr->targetVelocity = this->dataPtr->takeOffPower;
      this->dataPtr->targetPitch = std::max(0.1, this->dataPtr->takeOffMinPitch);
      this->dataPtr->rollControl.Reset();
      this->dataPtr->pitchControl.Reset();
    }
  }
  else if (
    this->dataPtr->mode == FixedWingControllerPrivate::Mode::TAKE_OFF_CLIMB)
  {
    /// Maintain a fixed climb rate
    this->dataPtr->targetVelocity = this->dataPtr->takeOffPower;
    this->dataPtr->targetPitch = std::max(0.1, this->dataPtr->takeOffMinPitch);
    this->dataPtr->ExecuteAttitudeControlLoop(rotation, _info);
    /// Once target altitude is reached level out.
    if (pose->Data().Z() >= this->dataPtr->takeOffAltitude) {
      this->dataPtr->mode = FixedWingControllerPrivate::Mode::ATTITUDE_CONTROL;
      this->dataPtr->targetPitch = 0;
      this->dataPtr->rollControl.Reset();
      this->dataPtr->pitchControl.Reset();
    }
  }
  else if (
    this->dataPtr->mode == FixedWingControllerPrivate::Mode::ATTITUDE_CONTROL)
  {
    this->dataPtr->ExecuteAttitudeControlLoop(rotation, _info);
  }

  // For Logging
  // auto currPitch = rotation.Dot(this->dataPtr->pitchAxis);
  // auto angVelocityComp =
  //   _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->entity);
  // auto linVelocityComp =
  //   _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->entity);
  // auto linVel = linVelocityComp->Data().Length();

  //this->dataPtr->logFile << currPitch << "," << linVel << ", " << cmd << "\n";
  //this->dataPtr->logFile.flush();

}

IGNITION_ADD_PLUGIN(
    mbzirc::FixedWingControllerPlugin,
    ignition::gazebo::System,
    mbzirc::FixedWingControllerPlugin::ISystemConfigure,
    mbzirc::FixedWingControllerPlugin::ISystemPreUpdate)
