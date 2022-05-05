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

#include <ignition/msgs/float_v.pb.h>
#include <ignition/transport/Node.hh>

#include <ros_ign_bridge/convert/std_msgs.hpp>

#include <radar_msgs/msg/radar_scan.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/// \brief Bridge for converting ign msgs to ros
class NaiveRadarBridge : public rclcpp::Node
{
  public:
    NaiveRadarBridge():
      Node("naive_radar_bridge")
    {
      // declare a parameter that lets user specifies the ign topic
      // which will be bridged
      this->declare_parameter<std::string>("topic", "radar/scan");
      this->get_parameter("topic", this->topic);

      // create the ros publisher
      // the topic name will be remapped by the bridge node
      this->rosPub = this->create_publisher<radar_msgs::msg::RadarScan>(
          "radar/scan", 10);

      // create a timer to periodically check for ros subscribers
      this->timer = this->create_wall_timer(100ms,
          std::bind(&NaiveRadarBridge::CheckSubscribers, this));
    }

  /// \brief Periodically check subcription count on the radar topic.
  /// If subscribers are present, we initiate subscription to the ignition
  /// topics, convert ign msgs to ros msgs, then publish to ros topics .
  private: void CheckSubscribers()
  {
    if (this->rosPub->get_subscription_count() > 0u)
      this->SensorConnect();
  }

  /// \brief Callback when ros subscriber connects to the ros topic
  private: void SensorConnect()
  {
    // return if we are already subscribed to the ign topic
    if (this->subscribed)
      return;
    this->ignNode.Subscribe(this->topic, &NaiveRadarBridge::OnData, this);
    this->subscribed = true;
  }

  /// \brief Callback when msgs arer received on the ign topic
  /// \param[in] _msg Float_V msgs with data arranged in the folowing format:
  /// [range, azimuth, elevation, range2, azimuth2, elevation2, ...]
  public: void OnData(const ignition::msgs::Float_V &_msg)
  {
    // unsubscribe from the ign topic if there are no more ros subscribers
    if (this->rosPub->get_subscription_count() == 0u && this->subscribed)
    {
      this->ignNode.Unsubscribe(this->topic);
      this->subscribed = false;
      return;
    }

    // make sure data is in the expected format
    if (_msg.data().size() % 3 != 0)
      return;

    // pack ros msg and publish
    // convert ign header to ros header
    radar_msgs::msg::RadarScan rosMsg;
    ros_ign_bridge::convert_ign_to_ros(_msg.header(), rosMsg.header);
    // fill radar return data
    for (unsigned int i = 0; i < _msg.data().size(); i+=3u)
    {
      float range = _msg.data(i);
      float azimuth = _msg.data(i+1);
      float elevation = _msg.data(i+2);
      radar_msgs::msg::RadarReturn returnMsg;
      returnMsg.range = range;
      returnMsg.azimuth = azimuth;
      returnMsg.elevation = elevation;
      rosMsg.returns.push_back(returnMsg);
    }

    this->rosPub->publish(rosMsg);
  }

  /// \brief timer with callback to check for publisher subscription count
  private: rclcpp::TimerBase::SharedPtr timer;

  /// \brief Sensor data topic
  private: std::string topic;

  /// \brief If we are subscribed to the ign topic
  private: bool subscribed{false};

  /// \brief Ignition transport node
  private: ignition::transport::Node ignNode;

  /// \brief ROS Publisher for the radar msgs
  private: rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr rosPub;
};

int main (int argc, const char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NaiveRadarBridge>());
  rclcpp::shutdown();
}
