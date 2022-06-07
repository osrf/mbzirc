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

#ifndef MBZIRC_SEED__UAV_CONTROLLER_HH_
#define MBZIRC_SEED__UAV_CONTROLLER_HH_

#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <ignition/math/PID.hh>

namespace mbzirc_seed
{

class UavController : public rclcpp::Node
{
public:
  explicit UavController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void onAirPressure(const sensor_msgs::msg::FluidPressure & msg);
  void onImu(const sensor_msgs::msg::Imu & msg);
  void onMagneticField(const sensor_msgs::msg::MagneticField & msg);

  void onTimer();

protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr air_pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_sub_;

  rclcpp::TimerBase::SharedPtr controller_timer_;

  double x_vel;
  double y_vel;
  double currentPressure;
  double targetPressure;

  ignition::math::PID altitudeControl;
};

}  // namespace mbzirc_seed
#endif  // MBZIRC_SEED__UAV_CONTROLLER_HH_
