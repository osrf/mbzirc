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
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "helper/TestFixture.hh"

TEST_F(MBZIRCTestFixture, SpawnUAVTest)
{
  /// This test checks that the UAV is spawned correctly.
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "quadrotor"},
    {"world", "faster_than_realtime"},
    {"model", "mbzirc_quadrotor"},
    {"x", "1"},
    {"y", "2"},
    {"z", "0.05"},
    {"flightTime", "10"}
  };

  bool spawnedSuccessfully = false;

  SetMaxIter(10000);

  LoadWorld("faster_than_realtime.sdf");

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);

    /// Check for model
    auto modelEntity = world.ModelByName(_ecm, "quadrotor");
    if (modelEntity != ignition::gazebo::kNullEntity)
    {
      spawnedSuccessfully = true;
    }

    if (Iter() % 1000 == 0)
      ignmsg << Iter() <<std::endl;

  });

  StartSim();
  auto launchHandle = LaunchWithParams("spawn.launch.py", params);
  WaitForMaxIter();

  // Wait for a maximum of 50K iterations or till the vehicle has been spawned.
  while(!spawnedSuccessfully && Iter() < 50000)
  {
    Step(100);
  }
  StopLaunchFile(launchHandle);

  ASSERT_TRUE(spawnedSuccessfully);
}

TEST_F(MBZIRCTestFixture, TestBatteryDuration)
{
  // clean up test log dir
  std::string logPath = "mbzirc_logs";
  ignition::common::removeAll(logPath);

  using namespace std::literals::chrono_literals;

  /// This test checks that the UAV battery gets drained in one minute after
  /// setting the time to one minute.
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "quadrotor"},
    {"world", "faster_than_realtime"},
    {"model", "mbzirc_quadrotor"},
    {"x", "1"},
    {"y", "2"},
    {"z", "0.05"},
    {"flightTime", "0.5"}
  };

  ignition::transport::Node node;
  auto pub = node.Advertise<ignition::msgs::Twist>("/model/quadrotor/cmd_vel");

  bool spawnedSuccessfully = false, publisherReady = false;

  std::chrono::steady_clock::duration firstSpawned;
  std::chrono::steady_clock::duration timeNow;
  std::chrono::steady_clock::duration dischargedAt;

  bool fullyDischarged = false;

  // Monitor the discharge of the battery
  std::function<void(const ignition::msgs::BatteryState&)> batteryCb
    = [&](const ignition::msgs::BatteryState &_state) -> void
    {
      if (_state.percentage() <= 0 && !fullyDischarged)
      {
        dischargedAt = timeNow;
        fullyDischarged = true;
      }
    };
  node.Subscribe("/model/quadrotor/battery/linear_battery/state", batteryCb);

  SetMaxIter(10000);

  LoadWorld("faster_than_realtime.sdf");

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);

    auto modelEntity = world.ModelByName(_ecm, "quadrotor");
    if (modelEntity != ignition::gazebo::kNullEntity)
    {
      if (!spawnedSuccessfully)
      {
        spawnedSuccessfully = true;
      }
      else
      {
        // Sync the sim time with the battery listener thread.
        timeNow = _info.simTime;
      }

      if(!pub.HasConnections())
        return;
      // Send a command to ask the robot to fly this is to
      // trigger discharging of the battery.
      ignition::msgs::Twist twist;
      ignition::msgs::Vector3d* vec = new ignition::msgs::Vector3d;
      vec->set_x(0);
      vec->set_y(0);
      vec->set_z(1);
      twist.set_allocated_linear(vec);
      pub.Publish(twist);
      if(!publisherReady)
      {
        firstSpawned = _info.simTime;
        publisherReady = true;
      }
    }
  });

  StartSim();
  auto launchHandle = LaunchWithParams("spawn.launch.py", params);
  WaitForMaxIter();

  using namespace std::literals::chrono_literals;

  // Wait for the battery to fully discharge or up to 2 minutes in simulated
  // time.
  while(!fullyDischarged && (timeNow - firstSpawned < 2min))
  {
    Step(100);
  }

  StopLaunchFile(launchHandle);

  ASSERT_TRUE(spawnedSuccessfully);
  ASSERT_TRUE(fullyDischarged);
  ASSERT_NEAR(
    std::chrono::duration<double>(dischargedAt - firstSpawned).count(), 30, 5);

  // check that dead battery event is logged
  std::string eventsLogPath =
      ignition::common::joinPaths(logPath, "events.yml");
  EXPECT_TRUE(ignition::common::isFile(eventsLogPath));
  int sleep = 0;
  int maxSleep = 10;
  bool eventFound = false;
  while(!eventFound && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();
    eventFound = eventsBuffer.str().find("type: dead_battery") != std::string::npos
        && eventsBuffer.str().find("data: quadrotor") != std::string::npos;
  }
  EXPECT_TRUE(eventFound);
  ignition::common::removeAll(logPath);
}
