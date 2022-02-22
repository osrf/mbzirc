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

#include <chrono>
#include <gtest/gtest.h>

#include <ignition/msgs.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/attitude_target.hpp>

#include "helper/TestFixture.hh"

using namespace std::chrono_literals;

TEST_F(MBZIRCTestFixture, FixedWingController)
{
  // Create a ROS Node to commandeer the fixed wing
  rclcpp::init(0, nullptr);
  auto rosNode = std::make_shared<rclcpp::Node>("TestFixedWing");
  auto publisher =
    rosNode->create_publisher<mavros_msgs::msg::AttitudeTarget>("/zephyr/cmd/attitude", 10);
  auto timer_ = rosNode->create_wall_timer(100ms, [&]() {
    mavros_msgs::msg::AttitudeTarget msg;
    msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE
      | mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE
      | mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE;
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 1;
    msg.thrust = 1000;
    publisher->publish(msg);
  });

  std::vector<std::pair<std::string,std::string>> params{
    {"name", "zephyr"},
    {"world", "empty_platform"},
    {"model", "mbzirc_fixed_wing"},
    {"type",  "uav"},
    {"x", "15"},
    {"y", "0"},
    {"z", "10"}
  };
  auto launchHandle = LaunchWithParams("spawn.launch.py", params);

  bool spawnedSuccessfully = false;
  bool initialVelocitySet = false;
  bool startedSuccessfully = false;

   // Create a Gazebo world
  LoadWorld("empty_platform.sdf");

  OnPreUpdate([&](const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    /// Not yet spawned no velocity will be recorded
    if (!spawnedSuccessfully)
    {
      return;
    }

    if (!initialVelocitySet)
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "zephyr");
      if (modelEntity == ignition::gazebo::kNullEntity)
      {
        return;
      }

      auto modelVal = ignition::gazebo::Model(modelEntity);
      auto linkEntity = modelVal.LinkByName(_ecm, "wing");
      if (linkEntity == ignition::gazebo::kNullEntity)
      {
        ignerr << "Could not find link" << std::endl;
        ASSERT_TRUE(false);
        return;
      }
      auto linkVal = ignition::gazebo::Link(linkEntity);
      linkVal.EnableVelocityChecks(_ecm);
      initialVelocitySet = true;
    }
    rclcpp::spin_some(rosNode);
  });

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);

    /// Check for model
    auto modelEntity = world.ModelByName(_ecm, "zephyr");
    if (modelEntity != ignition::gazebo::kNullEntity)
    {
      spawnedSuccessfully = true;
    }

    /// Not yet spawned no velocity will be recorded
    if (!spawnedSuccessfully)
    {
      //ignerr << "No veh yet\n";
      return;
    }

    auto modelVal = ignition::gazebo::Model(modelEntity);
    auto linkEntity = modelVal.LinkByName(_ecm, "wing");
    if (linkEntity == ignition::gazebo::kNullEntity)
    {
      ignerr << "Could not find link" << std::endl;
      //ASSERT_TRUE(false);
      return;
    }
    auto linkVal = ignition::gazebo::Link(linkEntity);
    auto linearVel = linkVal.WorldLinearVelocity(_ecm);
    auto angularVel = linkVal.WorldAngularVelocity(_ecm);

    if(!linearVel.has_value())
    {
      ignerr << "No linear velocity\n";
      return;
    }

    if (linearVel->Length() > 0)
    {
      startedSuccessfully = true;
    }

    //igndbg << linearVel.value() << "\n";

    //auto worldPose = _ecm.Component<ignition::gazebo::components::WorldPose>(linkEntity);
    //if(!angularVel.has_value())
    //{
    //  ignerr << "No angular velocity\n";
    //  return;
    //}
    //igndbg << angularVel.value() << "\n";
    //igndbg << worldPose->Data().Rot().Euler() << "\n";
  });
  StartSim();
  WaitForMaxIter();

  while(!startedSuccessfully && Iter() < 500000)
  {
    Step(100);
  }

  StopLaunchFile(launchHandle);

  //ASSERT_TRUE(spawnedSuccessfully) << "Fixed Wing not spawned";
  //ASSERT_TRUE(startedSuccessfully) << "Model did not start moving";
}
