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
#include <ignition/gazebo/Model.hh>
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

  // summary log is updated every 1 seconds
  ignmsg << "Waiting 1 second for summary.yml to update" << std::endl;
  std::this_thread::sleep_for(1000ms);

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

TEST_F(MBZIRCTestFixture, GameLogicBoundaryPenalty)
{
  using namespace std::literals::chrono_literals;

  /// This test checks penalty is applied when robot moves out of bounds
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
  bool moveOutOfBounds = false;
  bool moveOutOfBoundsDone = false;
  bool moveWithinBounds = false;
  bool moveWithinBoundsDone = false;
  bool moveOutOfBounds2 = false;
  bool moveOutOfBounds2Done = false;
  SetMaxIter(1000);

  LoadWorld("faster_than_realtime.sdf");

  OnPreupdate([&](const ignition::gazebo::UpdateInfo &_info,
          ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);
    auto modelEntity = world.ModelByName(_ecm, "quadrotor");
    ignition::gazebo::Model model(modelEntity);
    if (moveOutOfBounds && !moveOutOfBoundsDone)
    {
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(0, -71, 100, 0, 0, 0));
      moveOutOfBoundsDone = true;
      ignmsg << "Moving UAV out of bounds." << std::endl;
    }
    else if (moveOutOfBounds2 && !moveOutOfBounds2Done)
    {
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(0, 0, 125, 0, 0, 0));
      moveOutOfBounds2Done = true;
      ignmsg << "Moving UAV out of bounds again." << std::endl;
    }
    else if (moveWithinBounds && !moveWithinBoundsDone)
    {
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(0, 0, 1, 0, 0, 0));
      moveWithinBoundsDone = true;
      ignmsg << "Moving UAV back within bounds." << std::endl;
    }
  });

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

  ASSERT_TRUE(spawnedSuccessfully);

  std::string logPath = "mbzirc_logs";
  std::string eventsLogPath =
      ignition::common::joinPaths(logPath, "events.yml");
  EXPECT_TRUE(ignition::common::isFile(eventsLogPath));
  std::string scoreLogPath =
      ignition::common::joinPaths(logPath, "score.yml");
  EXPECT_TRUE(ignition::common::isFile(scoreLogPath));

  // verify initial state of logs
  {
    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // events log should be empty
    EXPECT_TRUE(eventsBuffer.str().empty());
  }

  // score should be 0
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    EXPECT_EQ(0, std::stoi(scoreBuffer.str()));
  }

  // move robot out of bounds
  moveOutOfBounds = true;
  Step(1);

  // verify that exceed_boundary event is logged
  bool started = false;
  int sleep = 0;
  int maxSleep = 10;
  bool logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // there should only be one exceed_boundary event
    logged = eventsBuffer.str().find("type: exceed_boundary_1")
               != std::string::npos &&
             eventsBuffer.str().find("type: exceed_boundary_2")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // score should be time penalty (300)
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    EXPECT_EQ(300, std::stoi(scoreBuffer.str()));
  }

  // move robot back within bounds
  // this does not trigger any event
  moveWithinBounds = true;
  Step(2000);

  // verify event log is the same
  logged = false;
  sleep = 0;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // there should only be one exceed_boundary event
    logged = eventsBuffer.str().find("type: exceed_boundary_1")
               != std::string::npos &&
             eventsBuffer.str().find("type: exceed_boundary_2")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // score should be sim time + penalty (300s)
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    EXPECT_LT(300, std::stoi(scoreBuffer.str()));
  }


  // move robot back out of bounds again
  moveOutOfBounds2 = true;
  Step(1);

  // verify there are now 2 exceed_bondary events
  logged = false;
  sleep = 0;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // there should now be two exceed_boundary events
    logged = eventsBuffer.str().find("type: exceed_boundary_1")
               != std::string::npos &&
             eventsBuffer.str().find("type: exceed_boundary_2")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  // competition should be terminated due to 2nd exceed_boundary event
  // score should be a large number (exceed time limit)
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    EXPECT_LT(3600, std::stof(scoreBuffer.str()));
  }

  StopLaunchFile(launchHandle);

  ignition::common::removeAll(logPath);
  return;
}
