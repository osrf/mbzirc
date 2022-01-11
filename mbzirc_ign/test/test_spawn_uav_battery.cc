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

#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "helper/TestFixture.hh"

TEST_F(MBZIRCTestFixture, SpawnUAVTest)
{
  std::vector<std::pair<std::string,std::string>> params{
    {"name", "x3"},
    {"world", "faster_than_realtime"},
    {"model", "x3_c2"},
    {"type",  "uav"},
    {"x", "1"},
    {"y", "2"},
    {"z", "0.05"},
    {"flightTime", "10"}
  };

  bool spawnedSuccessfully = false;

  SetMaxIter(30000);

  LoadWorld("faster_than_realtime.sdf");

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto worldEntity = ignition::gazebo::worldEntity(_ecm);
    ignition::gazebo::World world(worldEntity);

    auto modelEntity = world.ModelByName(_ecm, "x3");
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
  StopLaunchFile(launchHandle);

  ASSERT_TRUE(spawnedSuccessfully);
}

#if 0
TEST(uherih, frejorj)
{
   ignition::common::Console::SetVerbosity(4);

  auto fixture = std::make_unique<ignition::gazebo::TestFixture>(
      ignition::common::joinPaths(
          std::string(PROJECT_SOURCE_PATH), "worlds", "faster_than_realtime.sdf"));

  int _iter = 0;

  const int maxIter{2000};

  std::mutex finishedSim;
  std::condition_variable cv;

  bool spawnedSuccessfully = false;

  fixture->OnPostUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
      {
        _iter++;

        if (_iter % 1000 == 0)
          ignmsg << _iter << std::endl;

        /// Check if model has been spawned
        auto worldEntity = ignition::gazebo::worldEntity(_ecm);
        ignition::gazebo::World world(worldEntity);

        auto modelEntity = world.ModelByName(_ecm, "x3");
        if (modelEntity != ignition::gazebo::kNullEntity)
        {
          spawnedSuccessfully = true;
        }

        if (_iter == maxIter - 1)
        {
          {
            std::lock_guard<std::mutex> lk(finishedSim);
          }
          cv.notify_all();
        }
      });

  fixture->Finalize();

  fixture->Server()->Run(false, maxIter, false);
  std::string cmd = std::string(
      "ros2 launch mbzirc_ign spawn.launch.py name:=x3 world:=faster_than_realtime model:=x3_c2 type:=uav x:=1 y:=2 z:=0.05 flightTime:=10");

  auto pid = launchProcess(cmd);

  std::unique_lock<std::mutex> lock(finishedSim);
  cv.wait(lock, [&_iter, &maxIter]
          { return _iter == maxIter - 1; });
  ASSERT_TRUE(spawnedSuccessfully);

  std::cerr << "Killing " << pid << std::endl;
  killpg(pid, SIGINT);
}
#endif