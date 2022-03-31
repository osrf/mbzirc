/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;

// code adapted from ros_ign_bridge

//////////////////////////////////////////////////
/// \brief A class for testing ROS topic subscription.
template<typename ROS_T>
class MyTestClass
{
  /// \brief Class constructor.

public:
  explicit MyTestClass(const std::string & _topic)
  {
    this->sub =
      node->create_subscription<ROS_T>(_topic, 1000,
      std::bind(&MyTestClass::Cb, this, _1));
  }

  /// \brief Member function called each time a topic update is received.

public:
  void Cb(const typename ROS_T::SharedPtr _msg)
  {
    this->callbackExecuted = true;
  }

  /// \brief Member variables that flag when the actions are executed.

public:
  bool callbackExecuted = false;

/// \brief ROS subscriber;

private:
  typename rclcpp::Subscription<ROS_T>::SharedPtr sub;
};

/// \brief Wait until a boolean variable is set to true for a given number
/// of times. This function calls ros::spinOnce each iteration.
/// \param[in out] _boolVar The bool variable.
/// \param[in] _sleepEach Time duration to wait between each retry.
/// \param[in] _retries The number of retries.
///
/// E.g.:
///   using namespace std::chrono_literals;
///   waitUntilBoolVar(myVar, 1ms, 500);
template<class Rep, class Period>
void waitUntilBoolVarAndSpin(
  std::shared_ptr<rclcpp::Node> & node,
  bool & _boolVar,
  const std::chrono::duration<Rep, Period> & _sleepEach,
  const int _retries)
{
  int i = 0;
  while (!_boolVar && i < _retries) {
    ++i;
    std::this_thread::sleep_for(_sleepEach);
    rclcpp::spin_some(node);
  }
}

/////////////////////////////////////////////////
TEST(RosApiTest, SimTopics)
{
  // tf
  MyTestClass<tf2_msgs::msg::TFMessage> tf("/tf");
  waitUntilBoolVarAndSpin(
    node, tf.callbackExecuted, 10ms, 3500);
  EXPECT_TRUE(tf.callbackExecuted);

  // tf_static
  MyTestClass<tf2_msgs::msg::TFMessage> tfStatic("/tf_static");
  waitUntilBoolVarAndSpin(
    node, tfStatic.callbackExecuted, 10ms, 3500);
  EXPECT_TRUE(tfStatic.callbackExecuted);

  // score
  MyTestClass<std_msgs::msg::Float32> score("/mbzirc/score");
  waitUntilBoolVarAndSpin(
    node, score.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(score.callbackExecuted);

  // phase
  MyTestClass<std_msgs::msg::String> phase("/mbzirc/phase");
  waitUntilBoolVarAndSpin(
    node, phase.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(phase.callbackExecuted);

  // run_clock
  MyTestClass<rosgraph_msgs::msg::Clock> runClock("/mbzirc/run_clock");
  waitUntilBoolVarAndSpin(
    node, runClock.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(runClock.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(RosApiTest, UAVTopics)
{
  // imu
  MyTestClass<sensor_msgs::msg::Imu> imu(
      "/quadrotor/imu/data");
  waitUntilBoolVarAndSpin(
    node, imu.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(imu.callbackExecuted);

  // air_pressure
  MyTestClass<sensor_msgs::msg::FluidPressure> airPressure(
      "/quadrotor/air_pressure");
  waitUntilBoolVarAndSpin(
    node, airPressure.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(airPressure.callbackExecuted);

  // magnetometer
  MyTestClass<sensor_msgs::msg::MagneticField> magnetometer(
      "/quadrotor/magnetic_field");
  waitUntilBoolVarAndSpin(
    node, magnetometer.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(magnetometer.callbackExecuted);

  // rendering tests are disabled as it causes a crash on CI
  // \todo enable
  // image_raw
  MyTestClass<sensor_msgs::msg::Image> image(
      "/quadrotor/slot0/image_raw");
  waitUntilBoolVarAndSpin(
    node, image.callbackExecuted, 10ms, 3000);
  EXPECT_TRUE(image.callbackExecuted);

  // camera_info
  MyTestClass<sensor_msgs::msg::CameraInfo> cameraInfo(
      "/quadrotor/slot0/camera_info");
  waitUntilBoolVarAndSpin(
    node, cameraInfo.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(cameraInfo.callbackExecuted);

  // optical image
  MyTestClass<sensor_msgs::msg::Image> imageOptical(
      "/quadrotor/slot0/optical/image_raw");
  waitUntilBoolVarAndSpin(
    node, imageOptical.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(imageOptical.callbackExecuted);

  // optical camera_info
  MyTestClass<sensor_msgs::msg::CameraInfo> cameraInfoOptical(
      "/quadrotor/slot0/optical/camera_info");
  waitUntilBoolVarAndSpin(
    node, cameraInfoOptical.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(cameraInfoOptical.callbackExecuted);

  // rgbd - color
  MyTestClass<sensor_msgs::msg::Image> rgbdImage(
      "/hexrotor/slot0/image_raw");
  waitUntilBoolVarAndSpin(
    node, rgbdImage.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdImage.callbackExecuted);

  // rgbd - camera_info
  MyTestClass<sensor_msgs::msg::CameraInfo> rgbdCameraInfo(
      "/hexrotor/slot0/camera_info");
  waitUntilBoolVarAndSpin(
    node, rgbdCameraInfo.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdCameraInfo.callbackExecuted);

  // rgbd - depth
  MyTestClass<sensor_msgs::msg::Image> rgbdDepth(
      "/hexrotor/slot0/depth");
  waitUntilBoolVarAndSpin(
    node, rgbdDepth.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdDepth.callbackExecuted);

  // rgbd - points
  MyTestClass<sensor_msgs::msg::PointCloud2> rgbdPoints(
      "/hexrotor/slot0/points");
  waitUntilBoolVarAndSpin(
    node, rgbdPoints.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdPoints.callbackExecuted);

  // rgbd optical - color
  MyTestClass<sensor_msgs::msg::Image> rgbdOpticalImage(
      "/hexrotor/slot0/optical/image_raw");
  waitUntilBoolVarAndSpin(
    node, rgbdOpticalImage.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdOpticalImage.callbackExecuted);

  // rgbd optical - camera_info
  MyTestClass<sensor_msgs::msg::CameraInfo> rgbdOpticalCameraInfo(
      "/hexrotor/slot0/optical/camera_info");
  waitUntilBoolVarAndSpin(
    node, rgbdOpticalCameraInfo.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdOpticalCameraInfo.callbackExecuted);

  // rgbd optical - depth
  MyTestClass<sensor_msgs::msg::Image> rgbdOpticalDepth(
      "/hexrotor/slot0/optical/depth");
  waitUntilBoolVarAndSpin(
    node, rgbdOpticalDepth.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(rgbdOpticalDepth.callbackExecuted);

  // \todo check cmd_vel and points (lidar) topics
}

/////////////////////////////////////////////////
TEST(RosApiTest, ArmTopics)
{
  // joint states
  MyTestClass<sensor_msgs::msg::JointState> jointStates(
      "/joint_states");
  waitUntilBoolVarAndSpin(
    node, jointStates.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(jointStates.callbackExecuted);

  // image_raw
  MyTestClass<sensor_msgs::msg::Image> image(
      "/usv/arm/wrist_link/image_raw");
  waitUntilBoolVarAndSpin(
    node, image.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(image.callbackExecuted);

  // camera_info
  MyTestClass<sensor_msgs::msg::CameraInfo> cameraInfo(
      "/usv/arm/wrist_link/camera_info");
  waitUntilBoolVarAndSpin(
    node, cameraInfo.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(cameraInfo.callbackExecuted);

  // optical image
  MyTestClass<sensor_msgs::msg::Image> imageOptical(
      "/usv/arm/wrist_link/optical/image_raw");
  waitUntilBoolVarAndSpin(
    node, imageOptical.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(imageOptical.callbackExecuted);

  // optical camera_info
  MyTestClass<sensor_msgs::msg::CameraInfo> cameraInfoOptical(
      "/usv/arm/wrist_link/optical/camera_info");
  waitUntilBoolVarAndSpin(
    node, cameraInfoOptical.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(cameraInfoOptical.callbackExecuted);

  // gripper wrench
  MyTestClass<geometry_msgs::msg::Wrench> gripperLeftWrench(
      "/usv/arm/gripper/joint/finger_left/wrench");
  waitUntilBoolVarAndSpin(
    node, gripperLeftWrench.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(gripperLeftWrench.callbackExecuted);

  MyTestClass<geometry_msgs::msg::Wrench> gripperRightWrench(
      "/usv/arm/gripper/joint/finger_right/wrench");
  waitUntilBoolVarAndSpin(
    node, gripperRightWrench.callbackExecuted, 10ms, 500);
  EXPECT_TRUE(gripperRightWrench.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("test_ros_api");

  return RUN_ALL_TESTS();
}

