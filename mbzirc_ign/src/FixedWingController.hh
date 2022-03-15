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
#ifndef MBZIRC_IGN_FIXEDWING_HH_
#define MBZIRC_IGN_FIXEDWING_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace mbzirc
{
  class FixedWingControllerPrivate;

  /// \brief A plugin that takes care of fixed wing control of the zephyr.
  /// The controller is based on two PID loops: one to control the pitch,
  /// one to control the roll, and open loop forward velocity.
  /// This yaw rate in the zephyr is affected by the roll rate as the zephyr
  /// itself is underactuated.
  ///
  /// # Parameters
  /// `model_name` : The name of the rosnode (string)
  /// `cmd_topic` : ROS Topic on which users should publish commands to
  /// `link_name` : Link to be controlled (string)
  /// `roll_axis` : Axis in link frame which describes the roll (vector3d)
  /// `pitch_axis` : Axis in link frame which describes the pitch (vector3d)
  /// `roll_p` : Roll control P gain (double)
  /// `roll_i` : Roll control I gain (double)
  /// `roll_d` : Roll control D gain (double)
  /// `pitch_p` : Pitch control P gain (double)
  /// `pitch_i` : Pitch control I gain (double)
  /// `pitch_d` : Pitch control D gain (double)
  /// `left_flap`: Topic to publish to on ignition to control the
  ///    left flap joint position (string)
  /// `right_flap`: Topic to publish to on ignition to control the
  ///    right flap joint position (string)
  /// `thruster`: Topic to publish to on ignition to control the
  ///    thruster joint velocity (string)
  class FixedWingControllerPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: FixedWingControllerPlugin();

    /// \brief Destructor
    public: ~FixedWingControllerPlugin() override;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<FixedWingControllerPrivate> dataPtr;
  };
}
#endif
