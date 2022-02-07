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

#include <ignition/msgs.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "helper/TestFixture.hh"

TEST_F(MBZIRCTestFixture, USVMaxSpeedTest)
{
  /// This test checks that the USV is spawned correctly.
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "usv"},
    {"world", "faster_than_realtime"},
    {"model", "usv"},
    {"type",  "usv"},
    {"x", "15"},
    {"y", "0"},
    {"z", "-0.7"}
  };

  bool spawnedSuccessfully = false;
  bool initialVelocitySet = false;
  bool startedSuccessfully = false;

  ignition::transport::Node node;
  auto thruster_pub1 =
    node.Advertise<ignition::msgs::Double>(
      "/model/usv/joint/left_engine_propeller_joint/cmd_thrust");
  auto thruster_pub2 =
    node.Advertise<ignition::msgs::Double>(
      "/model/usv/joint/right_engine_propeller_joint/cmd_thrust");


  SetMaxIter(10000);

  LoadWorld("faster_than_realtime.sdf");

  OnPreUpdate([&](const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    /// Not yet spawned no velocity will be recorded
    if (!spawnedSuccessfully)
    {
      return;
    }

    // To speed up the test we give the vehicle an initial velocity of 4m/s
    if (!initialVelocitySet)
    {
      auto worldEntity = ignition::gazebo::worldEntity(_ecm);
      ignition::gazebo::World world(worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "usv");
      if (modelEntity == ignition::gazebo::kNullEntity)
      {
        return;
      }

      auto modelVal = ignition::gazebo::Model(modelEntity);
      auto linkEntity = modelVal.LinkByName(_ecm, "base_link");
      if (linkEntity == ignition::gazebo::kNullEntity)
      {
        ignerr << "Could not find link" << std::endl;
        ASSERT_TRUE(false);
        return;
      }
      auto linkVal = ignition::gazebo::Link(linkEntity);
      linkVal.EnableVelocityChecks(_ecm);
      linkVal.SetLinearVelocity(_ecm, ignition::math::Vector3d(4, 0, 0));
      initialVelocitySet = true;
    }
  });

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);

    /// Check for model
    auto modelEntity = world.ModelByName(_ecm, "usv");
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

    /// Publish an arbitrarily large thrust to the thrusters, it should be
    /// clamped down to a certain thrust
    ignition::msgs::Double thrust;
    thrust.set_data(1000000);

    thruster_pub1.Publish(thrust);
    thruster_pub2.Publish(thrust);

    /// Try to get the model speed
    auto modelVal = ignition::gazebo::Model(modelEntity);
    auto linkEntity = modelVal.LinkByName(_ecm, "base_link");
    if (linkEntity == ignition::gazebo::kNullEntity)
    {
      ignerr << "Could not find link" << std::endl;
      ASSERT_TRUE(false);
      return;
    }
    auto linkVal = ignition::gazebo::Link(linkEntity);
    auto linearVel = linkVal.WorldLinearVelocity(_ecm);

    ASSERT_TRUE(linearVel.has_value());
    if(!linearVel.has_value())
    {
      ignerr << "No linear velocity\n";
      return;
    }

    if (linearVel->Length() >0)
    igndbg << linearVel.value().X() << " "
           << linearVel.value().Y() << " "
           << linearVel.value().Z() << std::endl;

    /// Check that the model is moving
    if (linearVel->X() > 0.5)
    {
      startedSuccessfully = true;
    }

    /// Make sure that the model is moving forward at less than 8 knots
    ///ASSERT_TRUE(linearVel->X() < 8 * 0.5144)
    ///  << "Model is moving too fast " << linearVel->X() << "m/s\n";
  });

  StartSim();
  auto launchHandle = LaunchWithParams("spawn.launch.py", params);
  WaitForMaxIter();

  while(!startedSuccessfully && Iter() < 500000)
  {
    Step(100);
  }

  StopLaunchFile(launchHandle);

  ASSERT_TRUE(spawnedSuccessfully);
  //ASSERT_TRUE(startedSuccessfully);
}