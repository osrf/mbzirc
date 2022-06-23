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
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/transport/Node.hh>

#include "Components.hh"

#include "helper/TestFixture.hh"

TEST_F(MBZIRCTestFixture, CompetitorDetector)
{
  using namespace std::literals::chrono_literals;

  /// This test checks that the start and stop events are logged
  LoadWorld("faster_than_realtime.sdf");

  StartSim();

  std::atomic<int> model_count = 0;

  OnPostupdate([&](const ignition::gazebo::UpdateInfo &_info,
          const ignition::gazebo::EntityComponentManager &_ecm)
  {
    model_count = 0;
    _ecm.Each<mbzirc::components::CompetitorPlatform,
              ignition::gazebo::components::Model>(
        [&](const ignition::gazebo::Entity &_entity,
              const mbzirc::components::CompetitorPlatform *,
              const ignition::gazebo::components::Model *) -> bool
        {
          model_count++;
          return true;
        });
  });

  std::vector<std::pair<std::string,std::string>> params1{
    {"name", "quadrotor_1"},
    {"world", "faster_than_realtime"},
    {"model", "mbzirc_quadrotor"},
    {"x", "1"},
    {"y", "2"},
    {"z", "0.05"},
    {"flightTime", "10"},
    {"slot0", "mbzirc_vga_camera"},
    {"slot1", "mbzirc_hd_camera"},
    {"slot2", "mbzirc_planar_lidar"},
    {"slot3", "mbzirc_3d_lidar"},
    {"slot4", "mbzirc_rgbd_camera"}
  };

  EXPECT_EQ(0, model_count);

  SetMaxIter(1000);
  auto launchHandle1 = LaunchWithParams("spawn.launch.py", params1);
  WaitForMaxIter();
  EXPECT_EQ(1, model_count);

  StopLaunchFile(launchHandle1);

}
