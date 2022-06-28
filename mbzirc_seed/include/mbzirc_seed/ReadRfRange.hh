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

#ifndef MBZIRC_SEED__READ_RF_RANGE_HH_
#define MBZIRC_SEED__READ_RF_RANGE_HH_

#include <rclcpp/node.hpp>

#include <ros_ign_interfaces/msg/param_vec.hpp>

#include <unordered_map>

namespace mbzirc_seed
{

/// \brief Structure to represent an observation from the RF range sensor
struct RangeInfo
{
  /// \brief Platform name of the observation
  std::string platform;
  /// \brief Range to the platform
  double range;
  /// \brief RSSI of the measurement
  double rssi;
  /// \brief Time of the last update
  rclcpp::Time last_seen;
};

/// \brief Node to read RF ranging measurements from the MBZIRC simulation
class ReadRfRange : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit ReadRfRange(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /// \brief Callback for when air pressure messages are received
  void onRangeMessage(const ros_ign_interfaces::msg::ParamVec & msg);

  /// \brief Callback for when timer fires
  void onTimer();

  /// Subscriptions
  rclcpp::Subscription<ros_ign_interfaces::msg::ParamVec>::SharedPtr rf_range_sub_;

  /// Timers
  rclcpp::TimerBase::SharedPtr timer_;

  /// RF Range information
  std::mutex ranges_mutex_;
  std::unordered_map<std::string, RangeInfo> ranges_;
};
}  // namespace mbzirc_seed
#endif  // MBZIRC_SEED__READ_RF_RANGE_HH_
