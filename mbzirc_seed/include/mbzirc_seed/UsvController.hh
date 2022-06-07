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

#ifndef MBZIRC_SEED__USV_CONTROLLER_HH_
#define MBZIRC_SEED__USV_CONTROLLER_HH_

#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

namespace mbzirc_seed
{

class UsvController : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit UsvController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /// \brief Callback for when IMU messages are received
  void onImu(const sensor_msgs::msg::Imu & msg);

  /// \brief Callback for when periodic controller timer fires
  void onControllerTimer();

  /// Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;

  /// Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  /// Timers
  rclcpp::TimerBase::SharedPtr controller_timer_;
};

}  // namespace mbzirc_seed
#endif  // MBZIRC_SEED__USV_CONTROLLER_HH_
