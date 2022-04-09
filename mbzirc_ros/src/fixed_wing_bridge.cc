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
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

class FixedWingBridge : public rclcpp::Node
{
  public:
    FixedWingBridge():
      Node("fixed_wing_bridge")
    {
      this->declare_parameter<std::string>("model_name", "zephyr");
      this->get_parameter("model_name", model_name_);
      attitude_pub_ =
        ign_node_.Advertise<ignition::msgs::Float_V>(model_name_ + "/cmd/target_attitude");
      attitude_target_sub_ =
        this->create_subscription<mavros_msgs::msg::AttitudeTarget>("cmd/attitude", 10,
          std::bind(&FixedWingBridge::attitude_target_callback, this, std::placeholders::_1));
      command_tol_srv_ =
        this->create_service<mavros_msgs::srv::CommandTOL>("cmd/tol",
          std::bind(&FixedWingBridge::command_tol_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    void attitude_target_callback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg)
    {
      ignition::msgs::Float_V cmd_msg;
      cmd_msg.add_data(msg->orientation.w);
      cmd_msg.add_data(msg->orientation.x);
      cmd_msg.add_data(msg->orientation.y);
      cmd_msg.add_data(msg->orientation.z);
      cmd_msg.add_data(msg->thrust);

      attitude_pub_.Publish(cmd_msg);
    }

    void command_tol_callback(const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
      mavros_msgs::srv::CommandTOL::Response::SharedPtr res)
    {
      ignition::msgs::Float_V cmd_msg;
      ignition::msgs::Boolean resp;
      cmd_msg.add_data(req->min_pitch);
      cmd_msg.add_data(req->altitude);
      auto request = ign_node_.Request(
        model_name_ + "/cmd/takeoff",
        cmd_msg,
        10000000,
        resp,
        res->success
        );
    }
    ignition::transport::Node ign_node_;
    ignition::transport::Node::Publisher attitude_pub_;
    rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr
      attitude_target_sub_;
    rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr
      command_tol_srv_;
    std::string model_name_;
};

int main (int argc, const char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedWingBridge>());
  rclcpp::shutdown();
}
