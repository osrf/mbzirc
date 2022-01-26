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

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

#include <chrono>
#include <mutex>

#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include <sdf/sdf.hh>

#include "GameLogicPlugin.hh"

IGNITION_ADD_PLUGIN(
    mbzirc::GameLogicPlugin,
    ignition::gazebo::System,
    mbzirc::GameLogicPlugin::ISystemConfigure,
    mbzirc::GameLogicPlugin::ISystemPreUpdate,
    mbzirc::GameLogicPlugin::ISystemPostUpdate)

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace mbzirc;

class mbzirc::GameLogicPluginPrivate
{
  /// \brief Write a simulation timestamp to a logfile.
  /// \param[in] _simTime Current sim time.
  /// \return A file stream that can be used to write additional
  /// information to the logfile.
  public: std::ofstream &Log(const ignition::msgs::Time &_simTime);

  /// \brief Publish the score.
  /// \param[in] _event Unused.
  public: void PublishScore();

  /// \brief Finish game and generate log files
  /// \param[in] _simTime Simulation time.
  public: void Finish(const ignition::msgs::Time &_simTime);

  /// \brief Update the score.yml and summary.yml files. This function also
  /// returns the time point used to calculate the elapsed real time. By
  /// returning this time point, we can make sure that the ::Finish function
  /// uses the same time point.
  /// \param[in] _simTime Current sim time.
  /// \return The time point used to calculate the elapsed real time.
  public: std::chrono::steady_clock::time_point UpdateScoreFiles(
              const ignition::msgs::Time &_simTime);

  /// \brief Log an event to the eventStream.
  /// \param[in] _event The event to log.
  public: void LogEvent(const std::string &_event);

  /// \brief Battery subscription callback.
  /// \param[in] _msg Battery message.
  /// \param[in] _info Message information.
  public: void OnBatteryMsg(const ignition::msgs::BatteryState &_msg,
    const transport::MessageInfo &_info);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnStartCall(const ignition::msgs::Boolean &_req,
                            ignition::msgs::Boolean &_res);

  /// \brief Helper function to start the competition.
  /// \param[in] _simTime Simulation time.
  /// \return True if the run was started.
  public: bool Start(const ignition::msgs::Time &_simTime);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(const ignition::msgs::Boolean &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Current simulation time.
  public: ignition::msgs::Time simTime;

  /// \brief The simulation time of the start call.
  public: ignition::msgs::Time startSimTime;

  /// \brief Number of simulation seconds allowed.
  public: std::chrono::seconds runDuration{0};

  /// \brief Thread on which scores are published
  /// \brief Whether the task has started.
  public: bool started = false;

  /// \brief Whether the task has finished.
  public: bool finished = false;

  /// \brief Start time used for scoring.
  public: std::chrono::steady_clock::time_point startTime;

  /// \brief A mutex.
  public: std::mutex logMutex;

  /// \brief Log file output stream.
  public: std::ofstream logStream;

  /// \brief Event file output stream.
  public: std::ofstream eventStream;

  /// \brief Mutex to protect the eventStream.
  public: std::mutex eventMutex;

  /// \brief Ignition transport competition clock publisher.
  public: transport::Node::Publisher competitionClockPub;

  /// \brief Logpath.
  public: std::string logPath{"/dev/null"};

  /// \brief Names of the spawned robots.
  public: std::set<std::string> robotNames;

 /// \brief Current state.
  public: std::string state = "init";

  /// \brief Time at which the last status publication took place.
  public: std::chrono::steady_clock::time_point lastStatusPubTime;

  /// \brief Time at which the summary.yaml file was last updated.
  public: mutable std::chrono::steady_clock::time_point lastUpdateScoresTime;

  /// \brief Event manager for pausing simulation
  public: EventManager *eventManager{nullptr};

  /// \brief Models with dead batteries.
  public: std::set<std::string> deadBatteries;

  /// \brief Amount of allowed setup time in seconds.
  public: int setupTimeSec = 900;

  /// \brief Mutex to protect the eventCounter.
  public: std::mutex eventCounterMutex;

  /// \brief Counter to create unique id for events
  public: int eventCounter = 0;

  /// \brief Total score.
  public: double totalScore = 0.0;
};

//////////////////////////////////////////////////
GameLogicPlugin::GameLogicPlugin()
  : dataPtr(new GameLogicPluginPrivate)
{
}

//////////////////////////////////////////////////
GameLogicPlugin::~GameLogicPlugin()
{
  // set eventManager to nullptr so that Finish call does not attempt to
  // pause sim
  this->dataPtr->eventManager = nullptr;
  this->dataPtr->Finish(this->dataPtr->simTime);
}

//////////////////////////////////////////////////
void GameLogicPlugin::Configure(const ignition::gazebo::Entity & /*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager & /*_ecm*/,
                           ignition::gazebo::EventManager & _eventMgr)
{
  this->dataPtr->eventManager = &_eventMgr;

  // Check if the game logic plugin has a <logging> element.
  // example:
  // <logging>
  //   <path>/tmp</path>
  // </logging>
  const sdf::ElementPtr loggingElem =
    const_cast<sdf::Element *>(_sdf.get())->GetElement("logging");

  if (loggingElem)
  {
    // Get the logpath from the <path> element, if it exists.
    if (loggingElem->HasElement("path"))
    {
      this->dataPtr->logPath =
        loggingElem->Get<std::string>("path", "/dev/null").first;
    }
    else
    {
      // Make sure that we can access the HOME environment variable.
      char *homePath = getenv("HOME");
      if (!homePath)
      {
        ignerr << "Unable to get HOME environment variable. Report this error "
          << "to https://github.com/osrf/mbzirc/issues/new. "
          << "MBZIRC logging will be disabled.\n";
      }
      else
      {
        this->dataPtr->logPath = homePath;
      }
    }
  }

  ignmsg << "MBZIRC log path: " << this->dataPtr->logPath << std::endl;
  common::removeAll(this->dataPtr->logPath);
  common::createDirectories(this->dataPtr->logPath);

  // Open the log file.
  std::string filenamePrefix = "mbzirc";
  this->dataPtr->logStream.open(
      (common::joinPaths(this->dataPtr->logPath, filenamePrefix + "_" +
      ignition::common::systemTimeISO() + ".log")).c_str(), std::ios::out);

  // Open the event log file.
  this->dataPtr->eventStream.open(
      (common::joinPaths(this->dataPtr->logPath, "events.yml")).c_str(),
      std::ios::out);

  // Get the duration seconds.
  if (_sdf->HasElement("run_duration_seconds"))
  {
    this->dataPtr->runDuration = std::chrono::seconds(
        _sdf->Get<int>("run_duration_seconds"));

    ignmsg << "Run duration set to " << this->dataPtr->runDuration.count()
           << " seconds.\n";
  }

  // Get the duration seconds.
  if (_sdf->HasElement("setup_duration_seconds"))
  {
    this->dataPtr->setupTimeSec = std::chrono::seconds(
        _sdf->Get<int>("setup_duration_seconds")).count();

    ignmsg << "Setup duration set to " << this->dataPtr->setupTimeSec
           << " seconds.\n";
  }

  this->dataPtr->node.Advertise("/mbzirc/start",
      &GameLogicPluginPrivate::OnStartCall, this->dataPtr.get());

  this->dataPtr->node.Advertise("/mbzirc/finish",
      &GameLogicPluginPrivate::OnFinishCall, this->dataPtr.get());

  this->dataPtr->competitionClockPub =
    this->dataPtr->node.Advertise<ignition::msgs::Clock>("/mbzirc/run_clock");

  ignmsg << "Starting MBZIRC" << std::endl;

  // Make sure that there are score files.
  this->dataPtr->UpdateScoreFiles(this->dataPtr->simTime);
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnBatteryMsg(
    const ignition::msgs::BatteryState &_msg,
    const transport::MessageInfo &_info)
{
  ignition::msgs::Time localSimTime(this->simTime);
  if (_msg.percentage() <= 0)
  {
    std::vector<std::string> topicParts = common::split(_info.Topic(), "/");
    std::string name = "_unknown_";

    // Get the name of the model from the topic name, where the topic name
    // look like '/model/{model_name}/detach'.
    if (topicParts.size() > 1)
      name = topicParts[1];

    // Make sure the event is logged once.
    if (this->deadBatteries.find(name) == this->deadBatteries.end())
    {
      this->deadBatteries.emplace(name);
      {
        std::lock_guard<std::mutex> lock(this->eventCounterMutex);
        std::ostringstream stream;
        stream
          << "- event:\n"
          << "  id: " << this->eventCounter << "\n"
          << "  type: dead_battery\n"
          << "  time_sec: " << localSimTime.sec() << "\n"
          << "  robot: " << name << std::endl;

        this->LogEvent(stream.str());
      }
    }
  }
}

//////////////////////////////////////////////////
void GameLogicPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
}

//////////////////////////////////////////////////
void GameLogicPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Store sim time
  int64_t s, ns;
  std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);
  this->dataPtr->simTime.set_sec(s);
  this->dataPtr->simTime.set_nsec(ns);

  // Capture the names of the robots. We only do this until the team
  // triggers the start signal.
  if (!this->dataPtr->started)
  {
    _ecm.Each<gazebo::components::Sensor,
              gazebo::components::ParentEntity>(
        [&](const gazebo::Entity &,
            const gazebo::components::Sensor *,
            const gazebo::components::ParentEntity *_parent) -> bool
        {
          // Get the model. We are assuming that a sensor is attached to
          // a link.
          auto parent = _ecm.Component<gazebo::components::ParentEntity>(
              _parent->Data());
          auto model = parent;

          // find top level model
          while (parent && _ecm.Component<gazebo::components::Model>(
                 parent->Data()))
          {
            model = parent;
            parent = _ecm.Component<gazebo::components::ParentEntity>(
                parent->Data());
          }

          if (model)
          {
            // \todo(anyone)
            // Check if the robot has beyond the staging area.
            // we need to trigger the /mbzirc/start.

            // Get the model name
            auto mName =
              _ecm.Component<gazebo::components::Name>(model->Data());
            if (this->dataPtr->robotNames.find(mName->Data()) ==
                this->dataPtr->robotNames.end())
            {
              this->dataPtr->robotNames.insert(mName->Data());

              // Subscribe to battery state in order to log battery events.
              std::string batteryTopic = std::string("/model/") +
                mName->Data() + "/battery/linear_battery/state";
              this->dataPtr->node.Subscribe(batteryTopic,
                  &GameLogicPluginPrivate::OnBatteryMsg, this->dataPtr.get());
            }
          }
          return true;
        });

    // Start automatically if setup time has elapsed.
    if (this->dataPtr->simTime.sec() >= this->dataPtr->setupTimeSec)
    {
      this->dataPtr->Start(this->dataPtr->simTime);
    }
  }

  // Get the start sim time in nanoseconds.
  auto startSimTime = std::chrono::nanoseconds(
      this->dataPtr->startSimTime.sec() * 1000000000 +
      this->dataPtr->startSimTime.nsec());

  // Compute the elapsed competition time.
  auto elapsedCompetitionTime = this->dataPtr->started ?
    _info.simTime - startSimTime : std::chrono::seconds(0);

  // Compute the remaining competition time.
  auto remainingCompetitionTime = this->dataPtr->started &&
    this->dataPtr->runDuration != std::chrono::seconds(0) ?
    this->dataPtr->runDuration - elapsedCompetitionTime :
    std::chrono::seconds(0) ;

  // Check if the allowed time has elapsed. If so, then mark as finished.
  if ((this->dataPtr->started && !this->dataPtr->finished) &&
      this->dataPtr->runDuration != std::chrono::seconds(0) &&
      remainingCompetitionTime <= std::chrono::seconds(0))
  {
    ignmsg << "Time limit[" <<  this->dataPtr->runDuration.count()
      << "s] reached.\n";
    this->dataPtr->Finish(this->dataPtr->simTime);
  }

  auto currentTime = std::chrono::steady_clock::now();
  if (currentTime - this->dataPtr->lastStatusPubTime > std::chrono::seconds(1))
  {
    ignition::msgs::Clock competitionClockMsg;
    ignition::msgs::Header::Map *mapData =
      competitionClockMsg.mutable_header()->add_data();
    mapData->set_key("phase");
    if (this->dataPtr->started)
    {
      mapData->add_value(this->dataPtr->finished ? "finished" : "run");
      auto secondsRemaining = std::chrono::duration_cast<std::chrono::seconds>(
          remainingCompetitionTime);
      competitionClockMsg.mutable_sim()->set_sec(secondsRemaining.count());
    }
    else if (!this->dataPtr->finished)
    {
      mapData->add_value("setup");
      competitionClockMsg.mutable_sim()->set_sec(
          this->dataPtr->setupTimeSec - this->dataPtr->simTime.sec());
    }
    else
    {
      // It's possible for a team to call Finish before starting.
      mapData->add_value("finished");
    }

    this->dataPtr->competitionClockPub.Publish(competitionClockMsg);

    this->dataPtr->lastStatusPubTime = currentTime;
  }

  // Periodically update the score file.
  if (!this->dataPtr->finished && currentTime -
      this->dataPtr->lastUpdateScoresTime > std::chrono::seconds(30))
  {
    this->dataPtr->UpdateScoreFiles(this->dataPtr->simTime);
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishScore()
{
  transport::Node::Publisher scorePub =
    this->node.Advertise<ignition::msgs::Float>("/mbzirc/score");
  ignition::msgs::Float msg;

  while (!this->finished)
  {
    msg.set_data(this->totalScore);

    scorePub.Publish(msg);
    IGN_SLEEP_S(1);
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnFinishCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  ignition::msgs::Time localSimTime(this->simTime);
  if (this->started && _req.data() && !this->finished)
  {
    ignmsg << "User triggered OnFinishCall." << std::endl;
    this->Log(localSimTime) << "User triggered OnFinishCall." << std::endl;
    this->logStream.flush();

    this->Finish(localSimTime);
    _res.set_data(true);
  }
  else
    _res.set_data(false);

  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnStartCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  ignition::msgs::Time localSimTime(this->simTime);
  if (_req.data())
    _res.set_data(this->Start(localSimTime));
  else
    _res.set_data(false);

  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::Start(const ignition::msgs::Time &_simTime)
{
  bool result = false;

  if (!this->started && !this->finished)
  {
    result = true;
    this->started = true;
    this->startTime = std::chrono::steady_clock::now();
    this->startSimTime = _simTime;
    ignmsg << "Scoring has Started" << std::endl;
    this->Log(_simTime) << "scoring_started" << std::endl;
    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter << "\n"
        << "  type: started\n"
        << "  time_sec: " << _simTime.sec() << std::endl;
      this->LogEvent(stream.str());

      this->eventCounter++;
    }
  }

  // Update files when scoring has started.
  this->UpdateScoreFiles(_simTime);

  return result;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::Finish(const ignition::msgs::Time &_simTime)
{
  // Pause simulation when finished. Always send this request, just to be
  // safe.
  if (this->eventManager)
    this->eventManager->Emit<events::Pause>(true);

  if (this->finished)
    return;

  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;

  // Update the score.yml and summary.yml files. This function also
  // returns the time point used to calculate the elapsed real time. By
  // returning this time point, we can make sure that this function (the
  // ::Finish function) uses the same time point.
  std::chrono::steady_clock::time_point currTime =
    this->UpdateScoreFiles(_simTime);

  if (this->started)
  {
    realElapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currTime - this->startTime).count();

    simElapsed = _simTime.sec() - this->startSimTime.sec();

    ignmsg << "Scoring has finished. Elapsed real time: "
          << realElapsed << " seconds. Elapsed sim time: "
          << simElapsed << " seconds. " << std::endl;

    this->Log(_simTime) << "finished_elapsed_real_time " << realElapsed
      << " s." << std::endl;
    this->Log(_simTime) << "finished_elapsed_sim_time " << simElapsed
      << " s." << std::endl;
    this->Log(_simTime) << "finished_score " << this->totalScore << std::endl;
    this->logStream.flush();

    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter << "\n"
        << "  type: finished\n"
        << "  time_sec: " << _simTime.sec() << "\n"
        << "  elapsed_real_time: " << realElapsed << "\n"
        << "  elapsed_sim_time: " << simElapsed << "\n"
        << "  total_score: " << this->totalScore << std::endl;
      this->LogEvent(stream.str());
      this->eventCounter++;
    }
  }

  this->finished = true;
}

/////////////////////////////////////////////////
std::chrono::steady_clock::time_point GameLogicPluginPrivate::UpdateScoreFiles(
    const ignition::msgs::Time &_simTime)
{
  std::lock_guard<std::mutex> lock(this->logMutex);

  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;
  std::chrono::steady_clock::time_point currTime =
    std::chrono::steady_clock::now();

  if (this->started)
  {
    simElapsed = _simTime.sec() - this->startSimTime.sec();
    realElapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currTime - this->startTime).count();
  }

  // Output a run summary
  std::ofstream summary(this->logPath + "/summary.yml", std::ios::out);

  summary << "was_started: " << this->started << std::endl;
  summary << "sim_time_duration_sec: " << simElapsed << std::endl;
  summary << "real_time_duration_sec: " << realElapsed << std::endl;
  summary << "model_count: " << this->robotNames.size() << std::endl;
  summary.flush();

  // Output a score file with just the final score
  std::ofstream score(this->logPath + "/score.yml", std::ios::out);
  score << totalScore << std::endl;
  score.flush();

  this->lastUpdateScoresTime = currTime;
  return currTime;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::LogEvent(const std::string &_event)
{
  std::lock_guard<std::mutex> lock(this->eventMutex);
  this->eventStream << _event;
  this->eventStream.flush();
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPluginPrivate::Log(
    const ignition::msgs::Time &_simTime)
{
  this->logStream << _simTime.sec()
                  << " " << _simTime.nsec() << " ";
  return this->logStream;
}


