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

#include <ignition/common/Filesystem.hh>

#include <ignition/transport/Node.hh>

#include <ignition/msgs.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "helper/TestFixture.hh"

TEST_F(MBZIRCTestFixture, GameLogicTestStartStop)
{
  using namespace std::literals::chrono_literals;

  /// This test checks that the start and stop events are logged
  LoadWorld("faster_than_realtime.sdf");

  StartSim();

  std::string logPath = "mbzirc_logs";

  int sleep = 0;
  int maxSleep = 10;
  while(!ignition::common::exists(logPath) && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);
  }

  EXPECT_TRUE(ignition::common::exists(logPath));
  std::string eventsLogPath =
      ignition::common::joinPaths(logPath, "events.yml");
  EXPECT_TRUE(ignition::common::isFile(eventsLogPath));
  std::string summaryLogPath =
      ignition::common::joinPaths(logPath, "summary.yml");
  EXPECT_TRUE(ignition::common::isFile(summaryLogPath));

  // verify initial state of logs
  {
    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // events log should be empty
    EXPECT_TRUE(eventsBuffer.str().empty());

    // summary log should say the competition has not started yet
    std::ifstream summaryLog;
    summaryLog.open(summaryLogPath);
    std::stringstream summaryBuffer;
    summaryBuffer << summaryLog.rdbuf();
    EXPECT_NE(std::string::npos, summaryBuffer.str().find("was_started: 0"));
  }

  ignition::transport::Node node;

  unsigned int timeout = 5000;

  // publish finish event
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::Boolean req;
    req.set_data(true);
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/finish",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_FALSE(rep.data());
  }

  // Verify that competition is not finished because it has not been
  // started yet. Log content should be the same as initial state
  {
    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // events log should be empty
    EXPECT_TRUE(eventsBuffer.str().empty());

    // summary log should say the competition has not started yet
    std::ifstream summaryLog;
    summaryLog.open(summaryLogPath);
    std::stringstream summaryBuffer;
    summaryBuffer << summaryLog.rdbuf();
    EXPECT_NE(std::string::npos, summaryBuffer.str().find("was_started: 0"));
  }

  // publish start event
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::Boolean req;
    req.set_data(true);
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/start",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // verify that start event is logged
  bool started = false;
  sleep = 0;
  while(!started && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    std::ifstream summaryLog;
    summaryLog.open(summaryLogPath);
    std::stringstream summaryBuffer;
    summaryBuffer << summaryLog.rdbuf();

    started = eventsBuffer.str().find("type: started") != std::string::npos
        && summaryBuffer.str().find("was_started: 1") != std::string::npos;
  }

  EXPECT_TRUE(started);

  // publish finish event
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::Boolean req;
    req.set_data(true);
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/finish",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // verify that finish event is logged
  bool finished = false;
  sleep = 0;
  while(!finished && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    std::ifstream summaryLog;
    summaryLog.open(summaryLogPath);
    std::stringstream summaryBuffer;
    summaryBuffer << summaryLog.rdbuf();

    finished = eventsBuffer.str().find("type: finished") != std::string::npos
        && summaryBuffer.str().find("was_started: 1") != std::string::npos;
  }
  ignition::common::removeAll(logPath);
}

TEST_F(MBZIRCTestFixture, GameLogicPhase)
{
  using namespace std::literals::chrono_literals;

  /// This test checks that the competition run clock topic publishes
  // the right statuses
  LoadWorld("faster_than_realtime.sdf");

  StartSim();

  ignition::transport::Node node;

  // Run clock callback to keep track of current competition state
  std::mutex runClockMutex;
  std::string phaseData;
  std::function<void(const ignition::msgs::Clock &_msg)> runClockCb =
    [&](const ignition::msgs::Clock &_msg)
    {
      std::lock_guard<std::mutex> lock(runClockMutex);
      EXPECT_EQ(1, _msg.header().data_size());
      auto data = _msg.header().data(0);
      EXPECT_EQ(1, data.value_size());
      EXPECT_EQ("phase", data.key());
      phaseData = data.value(0);
    };
  node.Subscribe("/mbzirc/run_clock", runClockCb);

  // verify that initial phase is "setup"
  int sleep = 0;
  int maxSleep = 10;
  bool reachedPhase = false;
  while(!reachedPhase && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);
    Step(10);
    std::lock_guard<std::mutex> lock(runClockMutex);
    reachedPhase = phaseData == "setup";
  }
  EXPECT_EQ("setup", phaseData);

  // publish start event
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::Boolean req;
    req.set_data(true);
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/start",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // check that phase is now "run"
  sleep = 0;
  reachedPhase = false;
  while(!reachedPhase && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);
    Step(10);
    std::lock_guard<std::mutex> lock(runClockMutex);
    reachedPhase = phaseData == "run";
  }
  EXPECT_EQ("run", phaseData);

  // publish finish event
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::Boolean req;
    req.set_data(true);
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/finish",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // check that phase is now "finished"
  sleep = 0;
  reachedPhase = false;
  while(!reachedPhase && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);
    Step(10);
    std::lock_guard<std::mutex> lock(runClockMutex);
    reachedPhase = phaseData == "finished";
  }
  EXPECT_EQ("finished", phaseData);

  std::string logPath = "mbzirc_logs";
  ignition::common::removeAll(logPath);
}

TEST_F(MBZIRCTestFixture, GameLogicRobotModels)
{
  using namespace std::literals::chrono_literals;

  /// This test checks the robot count is correct in logs
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "quadrotor"},
    {"world", "faster_than_realtime"},
    {"model", "mbzirc_quadrotor"},
    {"type",  "uav"},
    {"x", "1"},
    {"y", "2"},
    {"z", "0.05"},
    {"flightTime", "10"}
  };

  bool spawnedSuccessfully = false;
  SetMaxIter(1000);

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

  // summary log is updated every 30 seconds
  ignmsg << "Waiting 30 seconds for summary.yml to update" << std::endl;
  std::this_thread::sleep_for(30000ms);

  // check that robot count in summary log is correct
  std::string logPath = "mbzirc_logs";
  std::string summaryLogPath =
      ignition::common::joinPaths(logPath, "summary.yml");
  EXPECT_TRUE(ignition::common::isFile(summaryLogPath));
  int sleep = 0;
  int maxSleep = 10;
  bool robotCountReached = false;
  while(!robotCountReached && sleep++ < maxSleep)
  {
    Step(1000);
    std::this_thread::sleep_for(1000ms);

    std::ifstream summaryLog;
    summaryLog.open(summaryLogPath);
    std::stringstream summaryBuffer;
    summaryBuffer << summaryLog.rdbuf();
    robotCountReached = summaryBuffer.str().find("model_count: 1")
        != std::string::npos;
  }
  EXPECT_TRUE(robotCountReached);

  ignition::common::removeAll(logPath);
  return;
}
