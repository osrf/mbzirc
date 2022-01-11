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


#ifndef _TESTFIXTURE_HH_
#define _TESTFIXTURE_HH_

#include <gtest/gtest.h>
#include <ignition/gazebo/TestFixture.hh>

#include "TestConstants.hh"

#include <unistd.h>

#define WRITE 0
#define READ 1

/// \brief Launch a command in a separate process group. This is needed to make
/// sure launch files and child processes actually exit.
/// \param[in] _command - The command to execute.
/// \returns The pgroup which the file is launched in.
pid_t launchProcess(std::string &_command)
{
  int p_stdin[2], p_stdout[2];
  pid_t pid;

  if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0)
    return -1;

  pid = fork();

  if (pid < 0)
    return pid;
  else if (pid == 0)
  {
    setpgid(0, 0);
    close(p_stdin[WRITE]);
    dup2(p_stdin[READ], READ);
    close(p_stdout[READ]);
    dup2(p_stdout[WRITE], WRITE);

    execl("/bin/sh", "sh", "-c", _command.c_str(), NULL);
    perror("execl");
    exit(1);
  }
  return pid;
}

/// \brief Convenient fixture that provides boilerplate code to common tests we
/// would like to run.
class MBZIRCTestFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    // Debug Everything
    ignition::common::Console::SetVerbosity(4);
  }

  /// \brief Loads the specified world. Note you should call this at the start
  /// of every test.
  public: void LoadWorld(std::string _world)
  {
    auto worldPath = ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", _world);

    // Setup fixture
    this->fixture = std::make_unique<ignition::gazebo::TestFixture>(worldPath);
  }

  /// \brief Sets the 
  public: void OnPostupdate(
    std::function<void(
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &)> func)
  {
    this->postUpdateFunc = func;
    this->fixture->OnPostUpdate(
      [&](const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
      {
        this->iter++;

        this->postUpdateFunc(_info, _ecm);

        if (this->iter == maxIter - 1)
        {
          {
            std::lock_guard<std::mutex> lk(finishedSim);
          }
          cv.notify_all();
        }
      });
  }

  public: void StartSim()
  {
    this->fixture->Finalize();
    this->fixture->Server()->Run(true, 1, false);
    this->fixture->Server()->Run(false, maxIter, false);
  }

  public: pid_t LaunchWithParams(
    const std::string &_file,
    const std::vector<std::pair<std::string, std::string>> &_params)
  {
    std::string cmd = "ros2 launch mbzirc_ign ";
    cmd += _file;

    for (auto &[arg, val]: _params)
    {
      std::string argPair = " " + arg + ":=" + val;
      cmd += argPair;
    }

    ignmsg << "Running : " << cmd;

    return launchProcess(cmd);
  }

  public: void StopLaunchFile(pid_t _launchfileHandle)
  {
    killpg(_launchfileHandle, SIGINT);
  }

  public: void SetMaxIter(int _iter)
  {
    this->maxIter = _iter;
  }

  public: void WaitForMaxIter()
  {
    std::unique_lock<std::mutex> lock(finishedSim);
    cv.wait(lock, [this]
      { return iter == maxIter - 1; });
  }

  public: int Iter()
  {
    return iter;
  }

  private: std::unique_ptr<ignition::gazebo::TestFixture> fixture;
 
  private: int maxIter{2000};

  private: std::mutex finishedSim;
  private: std::condition_variable cv;
  private: int iter{0};
  private: std::function<void(
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &)> postUpdateFunc;
};

#endif