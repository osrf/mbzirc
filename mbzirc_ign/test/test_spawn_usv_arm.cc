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

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <mutex>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/transport/Node.hh>

#include "helper/TestFixture.hh"

TEST_F(MBZIRCTestFixture, USVOberon7ArmGripper)
{
  /// This test checks that the USV oberon7 arm and gripper are
  /// spawned correctly and we can send joint pos commands to control
  /// them
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "usv"},
    {"world", "faster_than_realtime"},
    {"model", "usv"},
    {"type",  "usv"},
    {"x", "15"},
    {"y", "0"},
    {"z", "-0.7"},
    {"arm", "mbzirc_oberon7_arm"},
    {"gripper", "mbzirc_oberon7_gripper"}
  };

  bool spawnedSuccessfully = false;

  // create a list of joint pos command publishers for the arm and gripper
  ignition::transport::Node node;
  std::vector<std::string> armJointNames =
      {"azimuth", "shoulder", "elbow", "roll", "pitch", "wrist"};
  std::vector<std::string> gripperJointNames =
      {"finger_left", "finger_right"};
  std::vector<ignition::transport::Node::Publisher> armPublishers;
  for (auto joint : armJointNames)
  {
    auto pub = node.Advertise<ignition::msgs::Double>(
       "/usv/arm/" + joint);
    armPublishers.push_back(pub);
  }
  std::vector<ignition::transport::Node::Publisher> gripperPublishers;
  for (auto joint : gripperJointNames)
  {
    auto pub = node.Advertise<ignition::msgs::Double>(
       "/usv/arm/gripper/" + joint);
    gripperPublishers.push_back(pub);
  }

  SetMaxIter(10000);

  LoadWorld("faster_than_realtime.sdf");

  bool foundLinks = false;
  std::map<ignition::gazebo::Entity, ignition::math::Pose3d> armLinkPoses;
  std::vector<ignition::gazebo::Link> armLinks;
  std::map<ignition::gazebo::Entity, ignition::math::Pose3d> gripperLinkPoses;
  std::vector<ignition::gazebo::Link> gripperLinks;
  std::mutex mutex;

  // get all the arm and gripper links and poses in PostUpdate
  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (armLinks.empty())
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);

      /// Check for model
      auto modelEntity = world.ModelByName(_ecm, "usv");
      if (modelEntity == ignition::gazebo::kNullEntity)
      {
        return;
      }

      auto armEntity = _ecm.EntityByComponents(
        ignition::gazebo::components::ParentEntity(modelEntity),
        ignition::gazebo::components::Name("arm"),
        ignition::gazebo::components::Model());

      if (armEntity == ignition::gazebo::kNullEntity)
      {
        return;
      }

      auto gripperEntity = _ecm.EntityByComponents(
        ignition::gazebo::components::ParentEntity(armEntity),
        ignition::gazebo::components::Name("gripper"),
        ignition::gazebo::components::Model());

      if (gripperEntity == ignition::gazebo::kNullEntity)
      {
        return;
      }

      spawnedSuccessfully = true;

      std::vector<std::string> armLinkNames = {
          "shoulder_link", "upper_arm", "elbow_link", "forearm",
          "wrist_link"};

      std::vector<std::string> gripperLinkNames = {
          "finger_left", "finger_right"};

      auto arm = ignition::gazebo::Model(armEntity);
      for (auto link  : armLinkNames)
      {
        auto linkEntity = arm.LinkByName(_ecm, link);
        EXPECT_NE(ignition::gazebo::kNullEntity, linkEntity);
        armLinks.push_back(ignition::gazebo::Link(linkEntity));
      }
      EXPECT_EQ(armLinkNames.size(), armLinks.size());

      auto gripper = ignition::gazebo::Model(gripperEntity);
      for (auto link  : gripperLinkNames)
      {
        auto linkEntity = gripper.LinkByName(_ecm, link);
        EXPECT_NE(ignition::gazebo::kNullEntity, linkEntity);
        gripperLinks.push_back(ignition::gazebo::Link(linkEntity));
      }
      EXPECT_EQ(gripperLinkNames.size(), gripperLinks.size());
    }

    std::lock_guard<std::mutex> lock(mutex);
    // get arm poses
    for (auto &link  : armLinks)
    {
      auto comp = _ecm.Component<ignition::gazebo::components::Pose>(
          link.Entity());
      armLinkPoses[link.Entity()] = comp->Data();
    }

    // get gripper poses
    for (auto &link  : gripperLinks)
    {
      auto comp = _ecm.Component<ignition::gazebo::components::Pose>(
          link.Entity());
      gripperLinkPoses[link.Entity()] = comp->Data();

    }
  });

  StartSim();
  auto launchHandle = LaunchWithParams("spawn.launch.py", params);
  WaitForMaxIter();

  while(!spawnedSuccessfully && !foundLinks && Iter() < 500000)
  {
    Step(100);
  }

  // make sure we have the links
  EXPECT_EQ(armLinks.size(), armLinkPoses.size());
  EXPECT_EQ(gripperLinks.size(), gripperLinkPoses.size());

  Step(2000);

  // get initial link positions
  std::map<ignition::gazebo::Entity, ignition::math::Pose3d> armLinkPosesInit;
  std::map<ignition::gazebo::Entity, ignition::math::Pose3d>
      gripperLinkPosesInit;
  {
    std::lock_guard<std::mutex> lock(mutex);
    armLinkPosesInit = armLinkPoses;
    gripperLinkPosesInit = gripperLinkPoses;
  }

  // step and make sure arm is not moving
  Step(100);
  std::map<ignition::gazebo::Entity, ignition::math::Pose3d> armLinkPosesCopy;
  std::map<ignition::gazebo::Entity, ignition::math::Pose3d>
      gripperLinkPosesCopy;
  {
    std::lock_guard<std::mutex> lock(mutex);
    armLinkPosesCopy = armLinkPoses;
    gripperLinkPosesCopy = gripperLinkPoses;
  }

  // arm and gripper link positions should be the same as initial position
  for (auto & it : armLinkPosesCopy)
    EXPECT_EQ(armLinkPosesInit[it.first], it.second);
  for (auto & it : gripperLinkPosesCopy)
    EXPECT_EQ(gripperLinkPosesInit[it.first], it.second);

  // pub msg to move arm joints
  ignition::msgs::Double jointPosMsg;
  jointPosMsg.set_data(0.5);
  for (auto & pub : armPublishers)
    pub.Publish(jointPosMsg);
  for (auto & pub : gripperPublishers)
    pub.Publish(jointPosMsg);

  // wait a bit and step to make sure the arm moves
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1000ms);
  Step(2000);

  {
    std::lock_guard<std::mutex> lock(mutex);
    armLinkPosesCopy = armLinkPoses;
    gripperLinkPosesCopy = gripperLinkPoses;
  }

  // arm and gripper link positions should not be the same as initial position
  for (auto & it : armLinkPosesCopy)
    EXPECT_NE(armLinkPosesInit[it.first], it.second);
  for (auto & it : gripperLinkPosesCopy)
    EXPECT_NE(gripperLinkPosesInit[it.first], it.second);

  StopLaunchFile(launchHandle);

  ASSERT_TRUE(spawnedSuccessfully) << "USV not spawned";
}
