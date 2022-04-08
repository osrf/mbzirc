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

#include <sensor_msgs/msg/image.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

/// \brief A class that converts data from robot frame to optical frame
/// It subscribes to existing topic, modifies the frame_id of the message to
/// a new optical frame, then republishes the updated data to a new topic.
class VideoStreamPublisher : public rclcpp::Node
{
  /// \brief Constructor
  /// \param[in] _info True to publish camera info data in optical frame. False
  /// to publish only image data in optical frame
  public: VideoStreamPublisher(const std::string &_topic, const std::string &_frameId)
      : Node("video_stream_publisher")
  {
    this->frameId = _frameId;

    this->sub =
        this->create_subscription<sensor_msgs::msg::Image>(
        _topic, 10,
        std::bind(&VideoStreamPublisher::PublishTargetStream, this, _1));

    this->pub = this->create_publisher<sensor_msgs::msg::Image>(
        "/mbzirc/target/stream", 10);
  }

  /// \brief Subscriber callback which updates the frame id of the message
  /// and republishes the message.
  /// \param[in] _msg Message whose frame id is to be updated
  private: void PublishTargetStream(
      const std::shared_ptr<sensor_msgs::msg::Image> _msg)
  {
    auto m = *_msg.get();
    m.header.frame_id = this->frameId;
    this->pub->publish(m);
  }

  /// \brief Subscriber for the original image topic
  private: rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

  /// \brief Publisher for the image stream to be sent to base station
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;

  /// \brief Frame ID indicating which camera that the video stream is from
  private: std::string frameId;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (argc != 3)
  {
    std::cerr << "Usage: ros2 run mbzirc_ros video_stream_publisher "
              << "--ros-args [topic name] [camera frame ID]"
              << std::endl;
    exit(1);
  }

  // first arg is bool variable that specifies whether to re-publish camera info
  // in optical frame
  std::string topic = argv[1];
  std::string frameId = argv[2];

  rclcpp::spin(std::make_shared<VideoStreamPublisher>(topic, frameId));
  rclcpp::shutdown();
  return 0;
}
