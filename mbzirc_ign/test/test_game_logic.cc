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
#include <ignition/gazebo/components/Pose.hh>
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
  std::mutex phaseMutex;
  std::string phaseData;
  std::function<void(const ignition::msgs::StringMsg &_msg)> phaseCb =
    [&](const ignition::msgs::StringMsg &_msg)
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseData = _msg.data();
    };
  node.Subscribe("/mbzirc/phase", phaseCb);

  // verify that initial phase is "setup"
  int sleep = 0;
  int maxSleep = 10;
  bool reachedPhase = false;
  while(!reachedPhase && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);
    Step(10);
    std::lock_guard<std::mutex> lock(phaseMutex);
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

  // check that phase is now "started"
  sleep = 0;
  reachedPhase = false;
  while(!reachedPhase && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);
    Step(10);
    std::lock_guard<std::mutex> lock(phaseMutex);
    reachedPhase = phaseData == "started";
  }
  EXPECT_EQ("started", phaseData);

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
    std::lock_guard<std::mutex> lock(phaseMutex);
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
    Step(1);
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
    Step(1);
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
    Step(1);
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

TEST_F(MBZIRCTestFixture, GameLogicTargetReport)
{
  using namespace std::literals::chrono_literals;

  /// This test checks reporting targets over video stream
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "quadrotor"},
    {"world", "faster_than_realtime"},
    {"model", "mbzirc_quadrotor"},
    {"type",  "uav"},
    {"x", "1"},
    {"y", "2"},
    {"z", "1.05"},
    {"slot3", "mbzirc_hd_camera"},
    {"flightTime", "10"}
  };

  bool spawnedSuccessfully = false;
  bool moveAboveTargetVessel = false;
  bool moveAboveTargetVesselDone = false;
  bool moveAboveTargetSmallObject = false;
  bool moveAboveTargetSmallObjectDone = false;
  bool moveAboveTargetLargeObject = false;
  bool moveAboveTargetLargeObjectDone = false;

  SetMaxIter(1000);

  LoadWorld("faster_than_realtime.sdf");

  int simTimeSec = 0;
  OnPreupdate([&](const ignition::gazebo::UpdateInfo &_info,
          ignition::gazebo::EntityComponentManager &_ecm)
  {
    int64_t s, ns;
    std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);
    simTimeSec = s;

    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);
    auto modelEntity = world.ModelByName(_ecm, "quadrotor");
    ignition::gazebo::Model model(modelEntity);

    if (moveAboveTargetVessel && !moveAboveTargetVesselDone)
    {
      auto vesselEntity = world.ModelByName(_ecm, "Vessel A");
      auto poseComp =
          _ecm.Component<ignition::gazebo::components::Pose>(vesselEntity);
      ASSERT_TRUE(poseComp != nullptr);
      ignition::math::Pose3d pose = poseComp->Data();
      ignition::gazebo::Model vessel(vesselEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          pose.Pos() + ignition::math::Vector3d(0, 0, 80),
          ignition::math::Quaterniond::Identity));
      static bool output = false;
      if (!output)
      {
        ignmsg << "Moving UAV above target vessel." << std::endl;
        output = true;
      }
    }
    else if (moveAboveTargetSmallObject && !moveAboveTargetSmallObjectDone)
    {
      auto objectEntity = world.ModelByName(_ecm, "small_box_a");
      auto poseComp =
          _ecm.Component<ignition::gazebo::components::Pose>(objectEntity);
      ASSERT_TRUE(poseComp != nullptr);
      ignition::math::Pose3d pose = poseComp->Data();
      ignition::gazebo::Model object(objectEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          pose.Pos() + ignition::math::Vector3d(0, 0, 1),
          ignition::math::Quaterniond::Identity));
      static bool output = false;
      if (!output)
      {
        ignmsg << "Moving UAV above target small object." << std::endl;
        output = true;
      }
    }
    else if (moveAboveTargetLargeObject && !moveAboveTargetLargeObjectDone)
    {
      auto objectEntity = world.ModelByName(_ecm, "large_box_a");
      auto poseComp =
          _ecm.Component<ignition::gazebo::components::Pose>(objectEntity);
      ASSERT_TRUE(poseComp != nullptr);
      ignition::math::Pose3d pose = poseComp->Data();
      ignition::gazebo::Model object(objectEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          pose.Pos() + ignition::math::Vector3d(0, 0, 1),
          ignition::math::Quaterniond::Identity));
      static bool output = false;
      if (!output)
      {
        ignmsg << "Moving UAV above target large object." << std::endl;
        output = true;
      }
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

  // create callback for verifying competition phase
  std::string phase;
  ignition::transport::Node node;
  std::mutex phaseMutex;
  auto cb = [&](const ignition::msgs::StringMsg &_msg) -> void
  {
    std::lock_guard<std::mutex> lock(phaseMutex);
    phase = _msg.data();
  };

  // Subscribe to a topic by registering a callback.
  auto cbFunc = std::function<void(const ignition::msgs::StringMsg &)>(cb);
  EXPECT_TRUE(node.Subscribe("/mbzirc/phase", cbFunc));

  std::string logPath = "mbzirc_logs";
  std::string eventsLogPath =
      ignition::common::joinPaths(logPath, "events.yml");
  EXPECT_TRUE(ignition::common::isFile(eventsLogPath));
  std::string scoreLogPath =
      ignition::common::joinPaths(logPath, "score.yml");
  EXPECT_TRUE(ignition::common::isFile(scoreLogPath));

   // verify initial state of logs
  int score = 0;
  {
    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // events log should be empty
    EXPECT_TRUE(eventsBuffer.str().empty());

    // score should be 0
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    EXPECT_EQ(0, score);
  }

  // move quadrotor above target vessel
  moveAboveTargetVessel = true;
  Step(100);

  // verify that competition is started
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

    // there should be a started event
    logged = eventsBuffer.str().find("type: started")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  sleep = 0;
  bool phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "started";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);

  int startTime = simTimeSec;

  // start stream
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("quadrotor");
    req.add_data("sensor_3");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/start",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(10);

  // variables for counting score
  // score = sim time + time penlty
  // so keep track of sim time in OnPreUpdate and add penalties to it
  // to get expected score
  int vesselFirstPenalty = 180;
  int vesselSecondPenalty = 240;
  int smallFirstPenalty = 180;
  int smallSecondPenalty = 240;
  int largeFirstPenalty = 180;
  int largeSecondPenalty = 240;

  // verify that start stream request is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // there should be a stream_start_request event
    // but nothing should be reported yet
    logged = eventsBuffer.str().find("type: stream_start_request")
               != std::string::npos &&
             eventsBuffer.str().find("type: target_reported")
               == std::string::npos;

  }
  EXPECT_TRUE(logged);

  // get current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // test report target but give incorrect image position
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("vessel");
    req.add_data("240");
    req.add_data("480");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(100);

  // verify that a vessel report failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("type: target_reported_in_stream")
               != std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_failure_2")
               == std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // advance time
  Step(100);

  // test report target vessel but give incorrect image position again
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("vessel");
    req.add_data("340");
    req.add_data("480");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(100);

  // verify that another vessel report failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: vessel_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_failure_2")
               != std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // advance time
  Step(100);

  // test report correct target vessel
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("vessel");
    req.add_data("640");
    req.add_data("480");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(100);

  // verify that a vessel report success event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: vessel_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_failure_2")
               != std::string::npos &&
             eventsBuffer.str().find("data: vessel_id_success")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  sleep = 0;
  phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "vessel_id_success";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);

  // advance time
  Step(100);

  moveAboveTargetVesselDone = true;
  moveAboveTargetSmallObject = true;
  Step(20);

  // test report target small object but give incorrect image position
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("small");
    req.add_data("340");
    req.add_data("480");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(20);

  // verify that a small object report failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: small_object_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_id_failure_2")
               == std::string::npos &&
             eventsBuffer.str().find("data: small_object_id_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty +
        smallFirstPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // test report target small object but give incorrect image position again
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("small");
    req.add_data("280");
    req.add_data("280");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(20);

  // verify that another small object report failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: small_object_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_id_failure_2")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_id_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty +
        smallFirstPenalty + smallSecondPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // test report correct target small object
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("small");
    req.add_data("640");
    req.add_data("480");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(20);

  // verify that a small object report success event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: small_object_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_id_failure_2")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_id_success")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty +
        smallFirstPenalty + smallSecondPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // advance time
  Step(100);

  sleep = 0;
  phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "small_object_id_success";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);

  moveAboveTargetSmallObjectDone = true;
  moveAboveTargetLargeObject = true;
  Step(20);

  // test report target large object but give incorrect image position
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("large");
    req.add_data("0");
    req.add_data("0");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(20);

  // verify that a large object report failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: large_object_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_id_failure_2")
               == std::string::npos &&
             eventsBuffer.str().find("data: large_object_id_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty +
        smallFirstPenalty + smallSecondPenalty +
        largeFirstPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // test report target large object but give incorrect image position again
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("large");
    req.add_data("1280");
    req.add_data("960");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }
  Step(20);

  // verify that another large object report failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: large_object_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_id_failure_2")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_id_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty +
        smallFirstPenalty + smallSecondPenalty +
        largeFirstPenalty + largeSecondPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // test report correct large object
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg_V req;
    req.add_data("large");
    req.add_data("640");
    req.add_data("480");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/target/stream/report",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  Step(20);

  // verify that a large object report success event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("data: large_object_id_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_id_failure_2")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_id_success")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        vesselFirstPenalty + vesselSecondPenalty +
        smallFirstPenalty + smallSecondPenalty +
        largeFirstPenalty + largeSecondPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  moveAboveTargetLargeObjectDone = true;

  sleep = 0;
  phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "large_object_id_success";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);


  StopLaunchFile(launchHandle);
  ignition::common::removeAll(logPath);
}

TEST_F(MBZIRCTestFixture, GameLogicTargetRetrieval)
{
  using namespace std::literals::chrono_literals;

  /// This test checks reporting targets over video stream
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "usv"},
    {"world", "faster_than_realtime"},
    {"model", "usv"},
    {"type",  "usv"},
    {"x", "15"},
    {"y", "0"},
    {"z", "0.3"},
  };

  bool spawnedSuccessfully = false;
  bool dropSmallObjectAboveWater = false;
  bool dropSmallObjectAboveWaterDone = false;
  bool dropSmallObjectAboveUSV = false;
  bool dropSmallObjectAboveUSVDone = false;
  bool dropLargeObjectAboveWater = false;
  bool dropLargeObjectAboveWaterDone = false;
  bool dropLargeObjectAboveUSV = false;
  bool dropLargeObjectAboveUSVDone = false;
  SetMaxIter(1000);

  LoadWorld("faster_than_realtime.sdf");

  int simTimeSec = 0;
  bool paused = false;
  OnPreupdate([&](const ignition::gazebo::UpdateInfo &_info,
          ignition::gazebo::EntityComponentManager &_ecm)
  {
    paused = _info.paused;
    int64_t s, ns;
    std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);
    simTimeSec = s;

    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);
    if (dropSmallObjectAboveWater && !dropSmallObjectAboveWaterDone)
    {
      auto modelEntity = world.ModelByName(_ecm, "small_box_a");
      ignition::gazebo::Model model(modelEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          ignition::math::Vector3d(10, 10, 0.0),
          ignition::math::Quaterniond::Identity));
      dropSmallObjectAboveWaterDone = true;
      ignmsg << "Dropping small object above water." << std::endl;
    }
    if (dropSmallObjectAboveUSV && !dropSmallObjectAboveUSVDone)
    {
      auto modelEntity = world.ModelByName(_ecm, "small_box_a_duplicate");
      ignition::gazebo::Model model(modelEntity);
      auto usvEntity = world.ModelByName(_ecm, "usv");
      auto poseComp =
          _ecm.Component<ignition::gazebo::components::Pose>(usvEntity);
      ASSERT_TRUE(poseComp != nullptr);
      ignition::math::Pose3d pose = poseComp->Data();
      ignition::gazebo::Model usv(usvEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          pose.Pos() + ignition::math::Vector3d(0, 0, 1.0),
          ignition::math::Quaterniond::Identity));
      dropSmallObjectAboveUSVDone = true;
      ignmsg << "Dropping small object above USV." << std::endl;
    }
    else if (dropLargeObjectAboveWater && !dropLargeObjectAboveWaterDone)
    {
      auto modelEntity = world.ModelByName(_ecm, "large_box_a");
      ignition::gazebo::Model model(modelEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          ignition::math::Vector3d(10, -10, 0.0),
          ignition::math::Quaterniond::Identity));
      ignmsg << "Dropping large object above Water." << std::endl;
      dropLargeObjectAboveWaterDone = true;
    }
    else if (dropLargeObjectAboveUSV && !dropLargeObjectAboveUSVDone)
    {
      auto modelEntity = world.ModelByName(_ecm, "large_box_a_duplicate");
      ignition::gazebo::Model model(modelEntity);
      auto usvEntity = world.ModelByName(_ecm, "usv");
      auto poseComp =
          _ecm.Component<ignition::gazebo::components::Pose>(usvEntity);
      ASSERT_TRUE(poseComp != nullptr);
      ignition::math::Pose3d pose = poseComp->Data();
      ignition::gazebo::Model usv(usvEntity);
      model.SetWorldPoseCmd(_ecm, ignition::math::Pose3d(
          pose.Pos() + ignition::math::Vector3d(0, 0.4, 1.0),
          ignition::math::Quaterniond::Identity));
      ignmsg << "Dropping large object above USV." << std::endl;
      dropLargeObjectAboveUSVDone = true;
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

  // create callback for verifying competition phase
  std::string phase;
  ignition::transport::Node node;
  std::mutex phaseMutex;
  auto cb = [&](const ignition::msgs::StringMsg &_msg) -> void
  {
    std::lock_guard<std::mutex> lock(phaseMutex);
    phase = _msg.data();
  };

  // Subscribe to a topic by registering a callback.
  auto cbFunc = std::function<void(const ignition::msgs::StringMsg &)>(cb);
  EXPECT_TRUE(node.Subscribe("/mbzirc/phase", cbFunc));

  std::string logPath = "mbzirc_logs";
  std::string eventsLogPath =
      ignition::common::joinPaths(logPath, "events.yml");
  EXPECT_TRUE(ignition::common::isFile(eventsLogPath));
  std::string scoreLogPath =
      ignition::common::joinPaths(logPath, "score.yml");
  EXPECT_TRUE(ignition::common::isFile(scoreLogPath));

   // verify initial state of logs
  int score = 0;
  {
    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    // events log should be empty
    EXPECT_TRUE(eventsBuffer.str().empty());

    // score should be 0
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    EXPECT_EQ(0, score);
  }

  // skip to end of inspection phase / start of intervention phase
  {
    ignition::msgs::Boolean rep;
    ignition::msgs::StringMsg req;
    req.set_data("large_object_id_success");
    bool result = false;
    const unsigned int timeout = 5000;
    bool executed = node.Request("/mbzirc/skip_to_phase",
          req, timeout, rep, result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // verify that competition is started
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

    // there should be a started event
    logged = eventsBuffer.str().find("type: started")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  sleep = 0;
  bool phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "large_object_id_success";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);

  int startTime = simTimeSec;

  int smallObjectRetrievalPenalty = 120;
  int largeObjectRetrievalPenalty = 120;

  // drop small object above water
  dropSmallObjectAboveWater = true;
  Step(1500);

  // verify that a target object retrieval failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("type: target_retrieval")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_retrieve_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_retrieve_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        smallObjectRetrievalPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // advance time
  Step(100);

  // drop small object above usv
  dropSmallObjectAboveUSV = true;
  Step(100);

  // verify that a target object retrieval success event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("type: target_retrieval")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_retrieve_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: small_object_retrieve_success")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        smallObjectRetrievalPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // check that small target retrieve success phase is reached
  sleep = 0;
  phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "small_object_retrieve_success";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);

  // advance time
  Step(100);

  // drop large object above water
  dropLargeObjectAboveWater = true;
  Step(1500);

  // verify that a target object retrieval failure event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("type: target_retrieval")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_retrieve_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_retrieve_success")
               == std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        smallObjectRetrievalPenalty + largeObjectRetrievalPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // advance time
  Step(100);

  // drop large object above usv
  dropLargeObjectAboveUSV = true;

  // step until finished indicated by paused state
  // note: do not step multiple iterations otherwise we may run into inf loop
  int step = 0;
  while(!paused && ++step < 100)
    Step(1);

  // verify that a target object retrieval success event is logged
  sleep = 0;
  logged = false;
  while(!logged && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(1000ms);

    std::ifstream eventsLog;
    eventsLog.open(eventsLogPath);
    std::stringstream eventsBuffer;
    eventsBuffer << eventsLog.rdbuf();

    logged = eventsBuffer.str().find("type: target_retrieval")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_retrieve_failure_1")
               != std::string::npos &&
             eventsBuffer.str().find("data: large_object_retrieve_success")
               != std::string::npos;
  }
  EXPECT_TRUE(logged);

  // check current score
  {
    std::ifstream scoreLog;
    scoreLog.open(scoreLogPath);
    std::stringstream scoreBuffer;
    scoreBuffer << scoreLog.rdbuf();
    score = std::stoi(scoreBuffer.str());
    int expectedScore = simTimeSec - startTime +
        smallObjectRetrievalPenalty + largeObjectRetrievalPenalty;
    EXPECT_NEAR(expectedScore, score, 1);
  }

  // phase should now be finished
  sleep = 0;
  phaseReached = false;
  while(!phaseReached && sleep++ < maxSleep)
  {
    {
      std::lock_guard<std::mutex> lock(phaseMutex);
      phaseReached = phase == "finished";
    }
    Step(1);
    std::this_thread::sleep_for(1000ms);
  }
  EXPECT_TRUE(phaseReached);

  StopLaunchFile(launchHandle);
  ignition::common::removeAll(logPath);
}
