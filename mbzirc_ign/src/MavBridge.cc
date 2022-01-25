#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MavBridgeNode : public rclcpp::Node
{
public:
  MavBridgeNode() : Node("fixed_wing_bridge")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "topic", 10, std::bind(&MavBridgeNode::topic_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("topic ", "/model/zephyr/");
    std::string topic;
    this->get_parameter<std::string>("topic", topic);
    RCLCPP_INFO(this->get_logger(), "MavBridge publishing on '%s'",
      topic.c_str());
    pub_ = this->ign_node_.Advertise<ignition::msgs::Actuators>(topic);
  }

  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) const
  {
    ignition::msgs::Actuators actuator;
  }
private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  ignition::transport::Node ign_node_;
  ignition::transport::Node::Publisher pub_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MavBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
