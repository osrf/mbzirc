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

#include <mbzirc_seed/UsvController.hh>

namespace mbzirc_seed
{

UsvController::UsvController(const rclcpp::NodeOptions & options)
: rclcpp::Node("usv_controller", options)
{
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::QoS(10),
      std::bind(&UsvController::onImu, this, std::placeholders::_1));

  left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "left/thrust/cmd_thrust", rclcpp::QoS(10));
  left_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "left/thrust/joint/cmd_pos", rclcpp::QoS(10));

  right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "right/thrust/cmd_thrust", rclcpp::QoS(10));
  right_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "right/thrust/joint/cmd_pos", rclcpp::QoS(10));


  controller_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&UsvController::onControllerTimer, this));
}

void UsvController::onImu(const sensor_msgs::msg::Imu & /*msg*/)
{

}

void UsvController::onControllerTimer()
{
  std_msgs::msg::Float64 msg;
  msg.data = 100.0;
  left_thrust_pub_->publish(msg);
  right_thrust_pub_->publish(msg);
}

}  // namespace mbzirc_seed

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mbzirc_seed::UsvController)
