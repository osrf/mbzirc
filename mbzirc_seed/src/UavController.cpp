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

#include <mbzirc_seed/UavController.hh>

namespace mbzirc_seed
{

UavController::UavController(const rclcpp::NodeOptions & options)
: rclcpp::Node("uav_controller", options)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10));

  air_pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "air_pressure", rclcpp::QoS(10),
      std::bind(&UavController::onAirPressure, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::QoS(10),
      std::bind(&UavController::onImu, this, std::placeholders::_1));

  magnetic_field_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "magnetic_field", rclcpp::QoS(10),
      std::bind(&UavController::onMagneticField, this, std::placeholders::_1));

  this->declare_parameter<double>("altitude_p_gain", 0.005);
  this->declare_parameter<double>("altitude_i_gain", 0.000001);
  this->declare_parameter<double>("altitude_d_gain", 0);
  this->declare_parameter<double>("x_vel", 1.0);
  this->declare_parameter<double>("y_vel", 1.0);
  this->declare_parameter<double>("target_pressure", 101000.0);

  double p_gain, i_gain, d_gain;
  this->get_parameter("altitude_p_gain", p_gain);
  this->get_parameter("altitude_i_gain", i_gain);
  this->get_parameter("altitude_d_gain", d_gain);

  this->get_parameter("x_vel", x_vel);
  this->get_parameter("y_vel", y_vel);
  this->get_parameter("target_pressure", targetPressure);

  currentPressure = targetPressure;

  altitudeControl.SetPGain(p_gain);
  altitudeControl.SetIGain(i_gain);
  altitudeControl.SetDGain(d_gain);
  altitudeControl.SetCmdMax(4.0);
  altitudeControl.SetCmdMin(-4.0);

  controller_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&UavController::onControllerTimer, this));
}

void UavController::onControllerTimer()
{
  auto error = currentPressure - targetPressure;
  auto command = altitudeControl.Update(-error, std::chrono::milliseconds(100));
  geometry_msgs::msg::Twist msg;
  msg.linear.x = x_vel;
  msg.linear.y = y_vel;
  msg.linear.z = command;
  cmd_vel_pub_->publish(msg);
}

void UavController::onAirPressure(const sensor_msgs::msg::FluidPressure & msg)
{
  currentPressure = msg.fluid_pressure;
}

void UavController::onImu(const sensor_msgs::msg::Imu & /*msg*/)
{

}

void UavController::onMagneticField(const sensor_msgs::msg::MagneticField & /*msg*/)
{

}

}  // namespace mbzirc_seed

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mbzirc_seed::UavController)
