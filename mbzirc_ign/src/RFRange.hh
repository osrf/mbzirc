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
#ifndef MBZIRC_IGN_RFRANGE_HH_
#define MBZIRC_IGN_RFRANGE_HH_

#include <memory>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

using RFRangeType =
  ignition::gazebo::components::Component<ignition::gazebo::components::NoData,
                                          class RFRangeTypeTag>;
IGN_GAZEBO_REGISTER_COMPONENT("mbzirc_components.RFRangeType", RFRangeType)

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration.
  class RFRangePrivate;

  /// \brief A model plugin used to simulate a RF range sensor.
  ///
  /// Here's an example:
  /// <plugin
  ///   filename="libRFRange.so"
  ///   name="ignition::gazebo::systems::RFRangeSensor">
  /// </plugin>
  class RFRangeSensor
      : public System,
        public ISystemConfigure
  {
    /// \brief Constructor.
    public: RFRangeSensor();

    /// \brief Destructor.
    public: ~RFRangeSensor();

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;
  };

  /// \brief A world system used to compute the range between all pairs of
  /// RFRange sensors. This plugin builds on the same principles used in the
  /// RFComms system.
  ///
  /// \ref https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/rf_comms/RFComms.hh
  ///
  /// This system can be configured with the following SDF parameters:
  ///
  /// * Optional parameters:
  /// <update_rate> Sensor update rate (Hz).
  /// <range_config> Element used to capture the range configuration based on a
  ///                log-normal distribution. This block can contain any of the
  ///                next parameters:
  ///    * <max_range>: Hard limit on range (meters). No communication will
  ///                   happen beyond this range. Default is 50.
  ///    * <fading_exponent>: Fading exponent used in the normal distribution.
  ///                         Default is 2.5.
  ///    * <l0>: Path loss at the reference distance (1 meter) in dBm.
  ///            Default is 40.
  ///    * <sigma>: Standard deviation of the normal distribution.
  ///               Default is 10.
  ///    * <rssi_1>: RSSI value measured at the reference distance (1 meter) in
  ///                dBm.
  ///
  /// <radio_config> Element used to capture the radio configuration.
  ///                This block can contain any of the next parameters:
  ///    * <tx_power>: Transmitter power in dBm. Default is 27dBm (500mW).
  ///    * <noise_floor>: Noise floor in dBm.  Default is -90dBm.
  ///
  /// Here's an example:
  /// <plugin
  ///   filename="libRFRange.so"
  ///   name="ignition::gazebo::systems::RFRange">
  ///   <update_rate>1</update_rate>
  ///   <range_config>
  ///     <max_range>500000.0</max_range>
  ///     <fading_exponent>2.6</fading_exponent>
  ///     <l0>40</l0>
  ///     <sigma>10.0</sigma>
  ///     <rssi_1>-15</rssi_1>
  ///   </range_config>
  ///   <radio_config>
  ///     <tx_power>25</tx_power>
  ///     <noise_floor>-90</noise_floor>
  ///   </radio_config>
  /// </plugin>
  class RFRange
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: RFRange();

    /// \brief Destructor.
    public: ~RFRange() override = default;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const gazebo::UpdateInfo &_info,
                gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<RFRangePrivate> dataPtr;
  };
  }
}
}
}

#endif
