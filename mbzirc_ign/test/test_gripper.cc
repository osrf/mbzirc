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

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "helper/TestFixture.hh"

class GripperTestFixture: public MBZIRCTestFixture
{
  public: std::mutex pos_mutex;
};

TEST_F(GripperTestFixture, GripperController)
{
  LoadWorld("test/suction_gripper.sdf");

  ignition::transport::Node node;

  auto gripper_pub =
    node.Advertise<ignition::msgs::Double>(
      "/model/suction_gripper/joint/suction_gripper_joint/0/cmd_pos");

  auto suction_pub = node.Advertise<ignition::msgs::Boolean>(
    "/gripper/suction_on");

  ignition::math::Vector3d pos, vel;
  OnPreUpdate([&](const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);
    auto modelEntity = world.ModelByName(_ecm, "object");
    ignition::gazebo::Model model(modelEntity);
    auto linkEntity = model.LinkByName(_ecm, "object_1");
    ignition::gazebo::enableComponent<ignition::gazebo::components::WorldPose>(_ecm, linkEntity);
    ignition::gazebo::enableComponent<ignition::gazebo::components::WorldLinearVelocity>(_ecm, linkEntity);
  });

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);
    auto modelEntity = world.ModelByName(_ecm, "object");
    ignition::gazebo::Model model(modelEntity);
    auto linkEntity = model.LinkByName(_ecm, "object_1");

    ignition::gazebo::Link link(linkEntity);
    auto optional = link.WorldPose(_ecm);
    auto velocity = link.WorldLinearVelocity(_ecm);
    if (optional.has_value())
    {
      if (!tornDown.load())
      {
        std::lock_guard<std::mutex> lock(pos_mutex);
        pos = optional->Pos();
      }
    }
    if (velocity.has_value())
    {
      if (!tornDown.load())
      {
        std::lock_guard<std::mutex> lock(pos_mutex);
        vel = velocity.value();
      }
    }
  });

  // Run the simulation synchronously.
  StartSim(false);
  // Move arm to pick up object
  ignition::msgs::Double gripperMsg;
  gripperMsg.set_data(1.57);
  gripper_pub.Publish(gripperMsg);
  Step(20000);
  //using namespace std::chrono_literals;
  //std::this_thread::sleep_for(2000ms);
  {
    //std::lock_guard<std::mutex> lock(pos_mutex);
    EXPECT_NEAR(pos.X(), 1, 1e-3);
    EXPECT_NEAR(pos.Y(), 0, 1e-2);
    igndbg << pos <<"\n";
  }

  // Move object
  igndbg << "Commanding object to be \n";
  gripperMsg.set_data(0);
  gripper_pub.Publish(gripperMsg);
  Step(5000);

  // Check if object was moved by arm
  {
    //std::lock_guard<std::mutex> lock(pos_mutex);
    EXPECT_LT(pos.X(), 1);
    EXPECT_LT(pos.Y(), 0);
    igndbg << pos <<"\n";

  }

  igndbg << "Detaching gripper\n";
  // Detach gripper and swing the arm
  ignition::msgs::Boolean suction;
  suction.set_data(false);
  suction_pub.Publish(suction);
  //std::this_thread::sleep_for(1000ms);
  gripperMsg.set_data(-1.57);
  gripper_pub.Publish(gripperMsg);

  Step(3000);

  {
    // Object should be still as its been dropped off
    //std::lock_guard<std::mutex> lock(pos_mutex);
    EXPECT_NEAR(vel.Length(), 0, 1e-3);
        igndbg << pos <<"\n";

  }
}