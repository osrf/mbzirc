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

#include <memory>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ros_ign_interfaces/msg/string_vec.hpp>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/transport/Node.hh>

using std::placeholders::_1;

/// \brief A relay node for forward target and video streams to the
/// base station over inter-robot comms
class VideoTargetRelay : public rclcpp::Node
{
  /// \brief Constructor
  public: VideoTargetRelay();

  /// \brief Destructor
  public: ~VideoTargetRelay();

  /// \brief Callabck when a target is reported
  /// \param[in] _msg String vector message with the following data:
  /// [<target_type>, <image_x_pos>, <image_y_pos>]
  public: void OnTarget(
      const std::shared_ptr<ros_ign_interfaces::msg::StringVec> _msg);

  /// \brief Save image to file
  /// \param[in] _msg Image to save
  public: void SaveImage(
      const std::shared_ptr<sensor_msgs::msg::Image> _msg);

  /// \brief Callback when a video stream is received
  /// \param[in] _msg Image message
  public: void OnVideo(
      const std::shared_ptr<sensor_msgs::msg::Image> _msg);

  /// \brief Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief Subscriber for the target identification topic
  private: rclcpp::Subscription<ros_ign_interfaces::msg::StringVec>::SharedPtr
      targetSub;

  /// \brief Subscriber for the video stream topic
  private: rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      videoSub;

  /// \brief Publisher to publish target and video msgs to the comms broker
  public: ignition::transport::Node::Publisher brokerPub;

  /// \brief Name of robot that this relay node is for
  public: std::string robotName;

  /// \brief Mutex to protect saving images
  public: std::mutex mutex;

  /// \brief True to save image
  public: bool saveImage = false;

  /// \brief String prefix of image filename to save
  public: std::string saveImagePrefix;
};

//////////////////////////////////////////////////
VideoTargetRelay::VideoTargetRelay()
  : Node("video_target_relay")
{
  this->declare_parameter<std::string>("model_name", "vehicle");
  this->get_parameter("model_name", this->robotName);

  this->targetSub =
     this->create_subscription<ros_ign_interfaces::msg::StringVec>(
     "mbzirc/target/stream/report", 1,
     std::bind(&VideoTargetRelay::OnTarget, this, _1));

  this->videoSub =
     this->create_subscription<sensor_msgs::msg::Image>(
     "mbzirc/target/stream/start", 1,
     std::bind(&VideoTargetRelay::OnVideo, this, _1));

  std::string brokerTopic = "/broker/msgs";
  this->brokerPub = this->node.Advertise<ignition::msgs::Dataframe>(
      brokerTopic);
}

//////////////////////////////////////////////////
VideoTargetRelay::~VideoTargetRelay() = default;

//////////////////////////////////////////////////
void VideoTargetRelay::OnVideo(
    const std::shared_ptr<sensor_msgs::msg::Image> _msg)
{
  // frame_id should provide us info on which vehicle and sensor this
  // image is from
  std::string frameId = _msg->header.frame_id;
  ignition::msgs::StringMsg strMsg;
  strMsg.set_data(frameId);
  std::string data;
  strMsg.SerializeToString(&data);

  // pack the dataframe msg and send it to broker
  ignition::msgs::Dataframe msg;
  msg.set_src_address(this->robotName);
  msg.set_dst_address("base_station/video");
  msg.set_data(data);
  // send msg without image content as this is only for validating video
  // streaming rate
  this->brokerPub.Publish(msg);

  // save image
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->saveImage)
    {
      this->SaveImage(_msg);
      this->saveImage = false;
    }
  }
}

//////////////////////////////////////////////////
void VideoTargetRelay::SaveImage(
    const std::shared_ptr<sensor_msgs::msg::Image> _msg)
{
  cv::Mat image;
  std::string encoding = "bgr8";
  std::string path;
  ignition::common::env(IGN_HOMEDIR, path);

  std::string dir = ignition::common::joinPaths(
    path, ".ros", "mbzirc");
  if (!ignition::common::exists(dir))
  {
    ignition::common::createDirectories(dir);
  }

  std::string filename = ignition::common::joinPaths(dir,
      this->saveImagePrefix + ".png");
  RCLCPP_INFO(this->get_logger(), "Save target report image to %s.",
      filename.c_str());
  try
  {
    image = cv_bridge::toCvShare(_msg, encoding)->image;
  }
  catch (const cv_bridge::Exception &)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Unable to convert %s image to %s",
      _msg->encoding.c_str(), encoding.c_str());
    return;
  }
  if (!image.empty())
  {
    try
    {
      cv::imwrite(filename, image);
    }
    catch (...)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to save image %s",
        filename.c_str());
      return;
    }
  }
}

//////////////////////////////////////////////////
void VideoTargetRelay::OnTarget(
    const std::shared_ptr<ros_ign_interfaces::msg::StringVec> _msg)
{
  if (_msg->data.empty())
    return;

  ignition::msgs::Dataframe msg;
  msg.set_src_address(this->robotName);
  msg.set_dst_address("base_station/target");

  ignition::msgs::StringMsg_V strVMsg;
  for (const auto &elem : _msg->data)
  {
    strVMsg.add_data(elem);
  }

  if (!_msg->data.empty())
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->saveImagePrefix.clear();
    this->saveImagePrefix += _msg->data[0];
    this->saveImage = true;
  }
  std::string data;
  strVMsg.SerializeToString(&data);
  msg.set_data(data);

  this->brokerPub.Publish(msg);
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VideoTargetRelay>());
  rclcpp::shutdown();
  return 0;
}
