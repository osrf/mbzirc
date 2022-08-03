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

#include <ignition/math/Helpers.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/plugin/Register.hh>

#include <chrono>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include <ignition/math/AxisAlignedBox.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Image.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/rendering/RenderUtil.hh"
#include "ignition/gazebo/rendering/Events.hh"

#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/GpuLidar.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include <sdf/sdf.hh>

#include "GameLogicPlugin.hh"
#include "Components.hh"
#include "MbzircTypes.hh"

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
  /// \brief A map of penalty type and time penalties.
  public: const std::unordered_map<PenaltyType, int> kTimePenalties = {
          {PenaltyType::TARGET_VESSEL_ID_1, 180},
          {PenaltyType::TARGET_VESSEL_ID_2, 240},
          {PenaltyType::TARGET_VESSEL_ID_3, ignition::math::MAX_I32},
          {PenaltyType::SMALL_OBJECT_ID_1, 180},
          {PenaltyType::SMALL_OBJECT_ID_2, 240},
          {PenaltyType::SMALL_OBJECT_ID_3, ignition::math::MAX_I32},
          {PenaltyType::LARGE_OBJECT_ID_1, 180},
          {PenaltyType::LARGE_OBJECT_ID_2, 240},
          {PenaltyType::LARGE_OBJECT_ID_3, ignition::math::MAX_I32},
          {PenaltyType::SMALL_OBJECT_RETRIEVE_1, 120},
          {PenaltyType::SMALL_OBJECT_RETRIEVE_2, ignition::math::MAX_I32},
          {PenaltyType::LARGE_OBJECT_RETRIEVE_1, 120},
          {PenaltyType::LARGE_OBJECT_RETRIEVE_2, ignition::math::MAX_I32},
          {PenaltyType::BOUNDARY_1, 300},
          {PenaltyType::BOUNDARY_2, ignition::math::MAX_I32}};

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

  /// \brief Get the sim time msg
  /// \return Sim time msg
  public: ignition::msgs::Time SimTime();

  /// \brief Log an event to the eventStream.
  /// \param[in] _event The event to log.
  /// \param[in] _data Additional data for the event. Optional
  public: void LogEvent(const std::string &_event,
      const std::string &_data = "");

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

  /// \brief Ignition service callback triggered when finish is called.
  /// \param[in] _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(const ignition::msgs::Boolean &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition service callback triggered when targets are reported.
  /// \param[in] _req Report msg string in the following format:
  ///                 [vessel name, small obj name. large obj name]
  /// \param[out] _res The response message.
  public: bool OnReportTargets(const ignition::msgs::StringMsg_V &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition service callback triggered when video stream is to be
  /// started
  /// \param[in] _req Report msg string in the following format:
  ///                 [vehicle name, sensor_name]
  ///                 where vehicle and sensor name are the vehicle carrying
  ///                 the sensor with the image stream
  /// \param[out] _res The response message.
  public: bool OnTargetStreamStart(const ignition::msgs::StringMsg_V &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition service callback triggered when video stream is to be
  /// stopped.
  /// \param[in] _req Empty msg
  /// \param[out] _res The response message.
  public: bool OnTargetStreamStop(const ignition::msgs::Empty &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition service callback triggered when targets are reported
  /// with image position in the video stream
  /// \param[in] _req Report msg string in the following format:
  ///                 [type, image pos x, image pos y]
  ///                 where type can be "vessel", "small", or "large"
  /// \param[out] _res The response message.
  public: bool OnTargetStreamReport(const ignition::msgs::StringMsg_V &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition service for skipping to a particular phase. Used for
  /// testing and development.
  /// \param[in] _req String msg - phase to skip to.
  /// \param[out] _res The response message.
  /// \return True to indicate the service call is processed.
  public: bool OnSkipToPhase(const ignition::msgs::StringMsg &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Callback triggered when objects are placed on top
  /// of the USV
  /// \param[in] _msg The message containing name and pose of object placed
  public: void OnDetectObjectPlacement(const ignition::msgs::Pose &_msg);

  /// \brief Callback triggered when objects are dropped into the ocean
  /// \param[in] _msg The message containing name and pose of object placed
  public: void OnDetectObjectDropped(const ignition::msgs::Pose &_msg);

  /// \brief Check if an entity's position is within the input boundary
  //// \param[in] _ecm Entity component manager
  //// \param[in] _entity Entity id
  //// \param[in] _boundary Axis aligned bounding box
  public: bool CheckEntityInBoundary(const EntityComponentManager &_ecm,
                                     Entity _entity,
                                     const math::AxisAlignedBox &_boundary);


  /// \brief Pause trajectory following for the specified vessel
  /// \param[in] _vessel Name of vessel
  public: void PauseVesselTrajectory(const std::string &_vessel);

  /// \brief Detach target objects on the specified vessel
  /// \param[in] _vessel Name of vessel
  public: void DetachTargetObjects(const std::string &_vessel);

  /// \brief Valid target reports, add time penalties, and update score
  public: void ValidateTargetReports();

  /// \brief Valid intervention task and make sure target objects have been
  /// retrieved.
  public: void ValidateTargetObjectRetrieval();

  /// \brief Disable objects that are dropped in the ocean
  /// \param[in] _ecm Mutable reference to Entity Component Manager
  public: void DisableDroppedObjects(EntityComponentManager &_ecm);

  /// \brief Check task completion. Make sure all target objects have been
  /// retrieved
  public: void CheckTaskCompletion();

  /// \brief Find all competitor platforms and register them with game logic
  /// \param[in] Immutable reference to Entity Component Manager
  public: void EnumerateCompetitorPlatforms(const EntityComponentManager &_ecm);

  /// \brief Find all competitor sensors and register them with game logic
  /// \param[in] _entity model with sensors to enumerate
  /// \param[in] Immutable reference to Entity Component Manager
  public: void EnumerateCompetitorSensors(Entity _entity, const EntityComponentManager &_ecm);

  /// \brief Evalutate all competitor platforms and sensors for compliance
  public: bool AuditCompetitorConfiguration(const EntityComponentManager &_ecm);

  /// \brief Check if robots are inside geofence boundary. Time penalties are
  /// given if the robots exceed the first geofence two times, and the
  /// is terminated on the third occurence. If a robot moves outside of the
  /// outer geofence, the robot is disabled
  /// \param[in] _ecm Mutable reference to Entity Component Manager
  public: void CheckRobotsInGeofenceBoundary(
      EntityComponentManager &_ecm);

  /// \brief Make an entity static
  /// \param[in] _entity Entity to make static
  /// \param[in] _ecm Mutable reference to Entity Component Manager
  /// \return True if the entity is made static
  public: bool MakeStatic(Entity _entity, EntityComponentManager &_ecm);

  /// \brief Callback invoked in the rendering thread after a render update
  public: void OnPostRender();

  /// \brief Save image of target report
  /// \param[in] _type Type of target
  public: void SaveImage(const std::string &_type);

  /// \brief Find the visual of potential target at the specified image pos
  /// \param[in] _visual Target visual
  /// \param[in] _imagePos Image position to check for target
  /// \param[in] _type Type of target: vessel, small, or large
  /// \param[out] _objectAtImgPos Object found at image pos.
  /// \return True if a valid target is found
  public: bool FindTargetVisual(
      const rendering::VisualPtr &_visual,
      const math::Vector2i &_imagePos,
      const std::string &_type, std::string &_objectAtImgPos) const;

  /// \brief Check if visual is in camera view
  /// \param[in] _visual Visual to check
  /// \param[out] _imagePos 2D position of visual in the image
  /// \return True if visual is in view, false otherwise
  public: bool VisualInView(const rendering::VisualPtr &_visual,
      math::Vector2i &_imagePos) const;

  /// \brief Get visual at specified image position
  /// \param[in] _x X image position
  /// \param[in] _y Y image position
  /// \param[in] _type Type of target: vessel, small, or large
  /// \return Visual at the specified image position
  public: rendering::VisualPtr VisualAt(
      unsigned int _x, unsigned int _y, const std::string &_type) const;

  /// \brief Publish status message about stream requests / target reports
  /// \param[in] _status Status string to publish
  public: void PublishStreamStatus(const std::string &_status);

  /// \brief Set the competition phase
  /// \param[in] _phase Competition phase string
  public: void SetPhase(const std::string &_phase);

  /// \brief Get the competition phase
  /// \return Competition phase string
  public: std::string Phase();

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Current simulation time.
  public: ignition::msgs::Time simTime;

  /// \brief The simulation time of the start call.
  public: ignition::msgs::Time startSimTime;

  /// \brief Number of simulation seconds allowed.
  public: std::chrono::seconds runDuration{3600};

  /// \brief Thread on which scores are published
  public: std::unique_ptr<std::thread> scoreThread = nullptr;

  /// \brief Whether the task has started.
  public: bool started = false;

  /// \brief Whether the audit has been performed.
  public: bool audited = false;

  /// \brief Whether the task has finished.
  public: bool finished = false;

  /// \brief Start time (real time) used for scoring.
  public: std::chrono::steady_clock::time_point startTime;

  /// \brief Finish time (real time)
  public: std::chrono::steady_clock::time_point finishTime;

  /// \brief Mutex to protect log stream.
  public: std::mutex logMutex;

  /// \brief Mutex to protect sim time
  public: std::mutex simTimeMutex;

  /// \brief Log file output stream.
  public: std::ofstream logStream;

  /// \brief Event file output stream.
  public: std::ofstream eventStream;

  /// \brief Mutex to protect total score.
  public: std::mutex scoreMutex;

  /// \brief Mutex to protect reports
  public: std::mutex reportMutex;

  /// \brief Mutex to protect target stream.
  public: std::mutex streamMutex;

  /// \brief Mutex to protect phase.
  public: std::mutex phaseMutex;

  /// \brief Target reports
  public: std::vector<ignition::msgs::StringMsg_V> reports;

  /// \brief Object placements
  public: std::vector<ignition::msgs::Pose> objectPlacements;

  /// \brief Objects dropped
  public: std::vector<ignition::msgs::Pose> objectsDropped;

  /// \brief Objects that need to be disabled
  public: std::unordered_set<std::string> objectsToDisable;

  /// \brief Ignition transport competition clock publisher.
  public: transport::Node::Publisher competitionClockPub;

  /// \brief Ignition transport competition phase publisher.
  public: transport::Node::Publisher competitionPhasePub;

  /// \brief Ignition transport target stream status publisher.
  public: transport::Node::Publisher targetStreamStatusPub;

  /// \brief Logpath.
  public: std::string logPath{"/dev/null"};

  /// \brief target images path.
  public: std::string targetImagesPath{"/dev/null"};

  /// \brief Names of the spawned robots.
  public: std::unordered_map<Entity, PlatformInfo> robots;

  /// \brief Maximum number of rendering sensors across all platforms.
  public: int maximumSensors = {30};

  /// \brief Maximum number of usv platforms .
  public: int maximumUsvs = {1};

 /// \brief Current state.
  public: std::string state = "init";

  /// \brief Time at which the last status publication took place.
  public: std::chrono::steady_clock::time_point lastStatusPubTime;

  /// \brief Time at which the summary.yaml file was last updated.
  public: mutable std::chrono::steady_clock::time_point lastUpdateScoresTime;

  /// \brief Event manager for pausing simulation
  public: EventManager *eventManager{nullptr};

  /// \brief Models with dead batteries.
  public: std::unordered_set<std::string> deadBatteries;

  /// \brief Amount of allowed setup time in seconds.
  public: int setupTimeSec = 600;

  /// \brief Mutex to protect the eventCounter.
  public: std::mutex eventCounterMutex;

  /// \brief Counter to create unique id for events
  public: int eventCounter = 0;

  /// \brief Total score.
  public: double totalScore = ignition::math::INF_D;

  /// \brief boundary of competition area
  public: math::AxisAlignedBox geofenceBoundary;

  /// \brief Inner boundary of competition area
  /// This is geofenceBoundary - geofenceBoundaryBuffer
  public: math::AxisAlignedBox geofenceBoundaryInner;

  /// \brief Outer / hard boundary of competition area
  /// This is geofenceBoundary + geofenceBoundaryBufferOuter.
  /// If any vehicles moves outside of this boundary, it will be disabled
  public: math::AxisAlignedBox geofenceBoundaryOuter;

  /// \brief Inner boundary of competition area
  public: double geofenceBoundaryBuffer = 5.0;

  /// \brief Outer / hard boundary of competition area
  public: double geofenceBoundaryBufferOuter = 25.0;

  /// \brief boundary of the starting area
  public: math::AxisAlignedBox startingBoundary;

  /// \brief SDF DOM of a static model with empty link
  public: sdf::Model staticModelToSpawn;

  /// \brief Creator interface
  public: std::unique_ptr<SdfEntityCreator> creator{nullptr};

  /// \brief World entity
  public: Entity worldEntity{kNullEntity};

  /// \brief Number of times robot moved beyond competition boundary
  public: unsigned int geofenceBoundaryPenaltyCount = 0u;

  /// \brief Number of times vessel is incorrectly identified
  public: unsigned int vesselPenaltyCount = 0u;

  /// \brief Map of vessel to number of times small object is incorrectly
  /// identified
  public: std::unordered_map<std::string, unsigned int>
      smallObjectIdPenaltyCount;

  /// \brief Map of vessel to number times large object is incorrectly
  /// identified
  public: std::unordered_map<std::string, unsigned int>
      largeObjectIdPenaltyCount;

  /// \brief Map of vessel to number of times small object retrieval failed
  public: std::unordered_map<std::string, unsigned int>
      smallObjectRetrievePenaltyCount;

  /// \brief Map of vessel to number of times large object retrieval failed
  public: std::unordered_map<std::string, unsigned int>
      largeObjectRetrievePenaltyCount;

  /// \brief Total time penalty in seconds;
  public: int timePenalty = 0;

  /// \brief A map of target vessel name and targets.
  public: std::unordered_map<std::string, Target> targets;

  /// \brief Sensor entity on vehicle currently streaming video of target
  public: Entity targetStreamSensorEntity;

  /// \brief Keeps track of whether the sensor providing the target stream
  /// has changed or not.
  public: bool targetStreamSensorEntityChanged = false;

  /// \brief Name of topic that contains the video stream of target
  public: std::string targetStreamTopic;

  /// \brief Sensor entity on vehicle currently streaming video of target
  public: std::unordered_set<Entity> cameraSensors;

  /// \brief Connection to the post-render event.
  public: ignition::common::ConnectionPtr postRenderConn;

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Pointer to the rendering camera associated with the sensor
  /// providing the image stream
  public: rendering::CameraPtr camera;

  /// \brief Report of target in stream
  public: TargetInStream targetInStreamReport;

  /// \brief Current valid target vessel that has been successfully ID'ed.
  /// This variable is used later when validating reports of target objects.
  public: std::string currentTargetVessel;

  /// \brief A set of target vessel visuals
  public: std::unordered_set<rendering::VisualPtr> targetVesselVisuals;

  /// \brief A map of target vessel name and the small target object visuals
  /// on the vessel
  public: std::unordered_map<std::string, std::unordered_set<rendering::VisualPtr>>
      targetSmallObjectVisuals;

  /// \brief A map of target vessel name and the large target object visuals
  /// on the vessel
  public: std::unordered_map<std::string, std::unordered_set<rendering::VisualPtr>>
      targetLargeObjectVisuals;

  /// \brief Max allowed error (%) for reported target vessel image position
  public: const double kTargetVesselInImageTol = 0.03;

  /// \brief Max allowed error (%) for reported target object image position
  public: const double kTargetObjInImageTol = 0.008;

  /// \brief Compeition phase.
  public: std::string phase{kPhaseSetup};

  /// \brief Exit simulation process when run has finished.
  public: bool exitOnFinish{false};
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

  if (this->dataPtr->scoreThread)
    this->dataPtr->scoreThread->join();
}

//////////////////////////////////////////////////
void GameLogicPlugin::Configure(const ignition::gazebo::Entity & /*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager & _ecm,
                           ignition::gazebo::EventManager & _eventMgr)
{
  this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);
  this->dataPtr->worldEntity = _ecm.EntityByComponents(gazebo::components::World());
  this->dataPtr->eventManager = &_eventMgr;

  // Check if the game logic plugin has a <logging> element.
  // example:
  // <logging>
  //   <path>/tmp</path>
  // </logging>
  auto sdf = const_cast<sdf::Element *>(_sdf.get());
  const sdf::ElementPtr loggingElem = sdf->GetElement("logging");

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

  this->dataPtr->targetImagesPath =
      common::joinPaths(this->dataPtr->logPath, "target_images");
  common::createDirectories(this->dataPtr->targetImagesPath);

  // Open the log file.
  std::string filenamePrefix = "mbzirc";
  this->dataPtr->logStream.open(
      (common::joinPaths(this->dataPtr->logPath, filenamePrefix + "_" +
      ignition::common::systemTimeISO() + ".log")).c_str(), std::ios::out);

  // Open the event log file.
  this->dataPtr->eventStream.open(
      (common::joinPaths(this->dataPtr->logPath, "events.yml")).c_str(),
      std::ios::out);

  // Get the run duration seconds.
  if (_sdf->HasElement("run_duration_seconds"))
  {
    this->dataPtr->runDuration = std::chrono::seconds(
        _sdf->Get<int>("run_duration_seconds"));

    ignmsg << "Run duration set to " << this->dataPtr->runDuration.count()
           << " seconds.\n";
  }

  // Get competition geofence boundary.
  if (_sdf->HasElement("geofence"))
  {
    auto boundsElem = sdf->GetElement("geofence");

    if (boundsElem->HasElement("center") && boundsElem->HasElement("size"))
    {
      auto center = boundsElem->GetElement("center")->Get<math::Vector3d>();
      auto size = boundsElem->GetElement("size")->Get<math::Vector3d>();
      math::Vector3d max = center + size * 0.5;
      math::Vector3d min = center - size * 0.5;
      this->dataPtr->geofenceBoundary = math::AxisAlignedBox(min, max);
      this->dataPtr->geofenceBoundaryInner =
          math::AxisAlignedBox(min + this->dataPtr->geofenceBoundaryBuffer,
                               max - this->dataPtr->geofenceBoundaryBuffer);
      this->dataPtr->geofenceBoundaryOuter =
          math::AxisAlignedBox(min - this->dataPtr->geofenceBoundaryBufferOuter,
                               max + this->dataPtr->geofenceBoundaryBufferOuter);
      ignmsg << "Geofence boundary min: " << min << ", max: " << max << std::endl;
    }
    else
    {
      ignerr << "<geofence> is missing <center> and <size> SDF elements."
             << std::endl;
    }
  }

  // Get competition geofence boundary.
  if (_sdf->HasElement("start_area"))
  {
    auto boundsElem = sdf->GetElement("start_area");

    if (boundsElem->HasElement("center") && boundsElem->HasElement("size"))
    {
      auto center = boundsElem->GetElement("center")->Get<math::Vector3d>();
      auto size = boundsElem->GetElement("size")->Get<math::Vector3d>();
      math::Vector3d max = center + size * 0.5;
      math::Vector3d min = center - size * 0.5;
      this->dataPtr->startingBoundary = math::AxisAlignedBox(min, max);
      ignmsg << "Start area boundary min: " << min << ", max: " << max << std::endl;
    }
    else
    {
      ignerr << "<start_area> is missing <center> and <size> SDF elements."
             << std::endl;
    }
  }

  // Get the setup duration seconds.
  if (_sdf->HasElement("setup_duration_seconds"))
  {
    this->dataPtr->setupTimeSec = std::chrono::seconds(
        _sdf->Get<int>("setup_duration_seconds")).count();

    ignmsg << "Setup duration set to " << this->dataPtr->setupTimeSec
           << " seconds.\n";
  }

  // Get target vessels and objects
  if (_sdf->HasElement("target"))
  {
    auto targetElem = sdf->GetElement("target");
    while (targetElem)
    {
      if (targetElem->HasElement("vessel"))
      {
        mbzirc::Target target;

        // get target vessel name
        auto vesselTargetElem = targetElem->GetElement("vessel");
        std::string vessel = vesselTargetElem->Get<std::string>();
        target.vessel = vessel;
        ignmsg << "Target vessel: " << vessel << std::endl;;

        // get a list of small target objects on this vessel
        if (targetElem->HasElement("small_object"))
        {
          auto smallObjTargetElem = targetElem->GetElement("small_object");
          while (smallObjTargetElem)
          {
            std::string smallObj = smallObjTargetElem->Get<std::string>();
            target.smallObjects.insert(smallObj);
            smallObjTargetElem = smallObjTargetElem->GetNextElement("small_object");
            ignmsg << "  Small object: " << smallObj << std::endl;;
          }
        }

        // get a list of large target objects on this vessel
        if (targetElem->HasElement("large_object"))
        {
          auto largeObjTargetElem = targetElem->GetElement("large_object");
          while (largeObjTargetElem)
          {
            std::string largeObj = largeObjTargetElem->Get<std::string>();
            target.largeObjects.insert(largeObj);
            largeObjTargetElem = largeObjTargetElem->GetNextElement("large_object");
            ignmsg << "  Large object: " << largeObj << std::endl;;
          }
        }

        this->dataPtr->targets[vessel] = target;

      }
      else
      {
        ignerr << "<target> element must have a <vessel> child element" << std::endl;
      }

      targetElem = targetElem->GetNextElement("target");
    }
  }

  if (sdf->HasElement("exit_on_finish"))
  {
    this->dataPtr->exitOnFinish = sdf->Get<bool>("exit_on_finish");
  }

  this->dataPtr->node.Advertise("/mbzirc/start",
      &GameLogicPluginPrivate::OnStartCall, this->dataPtr.get());

  this->dataPtr->node.Advertise("/mbzirc/finish",
      &GameLogicPluginPrivate::OnFinishCall, this->dataPtr.get());

  this->dataPtr->node.Advertise("/mbzirc/report/targets",
      &GameLogicPluginPrivate::OnReportTargets, this->dataPtr.get());

  this->dataPtr->scoreThread.reset(new std::thread(
        &GameLogicPluginPrivate::PublishScore, this->dataPtr.get()));

  this->dataPtr->competitionClockPub =
    this->dataPtr->node.Advertise<ignition::msgs::Clock>("/mbzirc/run_clock");

  this->dataPtr->competitionPhasePub =
    this->dataPtr->node.Advertise<ignition::msgs::StringMsg>("/mbzirc/phase");

  this->dataPtr->targetStreamStatusPub =
      this->dataPtr->node.Advertise<ignition::msgs::StringMsg>(
      "/mbzirc/target/stream/status");

  this->dataPtr->node.Advertise("/mbzirc/target/stream/start",
      &GameLogicPluginPrivate::OnTargetStreamStart, this->dataPtr.get());

  this->dataPtr->node.Advertise("/mbzirc/target/stream/stop",
      &GameLogicPluginPrivate::OnTargetStreamStop, this->dataPtr.get());

  this->dataPtr->node.Advertise("/mbzirc/target/stream/report",
      &GameLogicPluginPrivate::OnTargetStreamReport, this->dataPtr.get());

  this->dataPtr->node.Advertise("/mbzirc/skip_to_phase",
      &GameLogicPluginPrivate::OnSkipToPhase, this->dataPtr.get());

  this->dataPtr->node.Subscribe("/mbzirc/target_object_detector/placed",
      &GameLogicPluginPrivate::OnDetectObjectPlacement, this->dataPtr.get());

  this->dataPtr->node.Subscribe("/mbzirc/target_object_detector/dropped",
      &GameLogicPluginPrivate::OnDetectObjectDropped, this->dataPtr.get());


  ignmsg << "Starting MBZIRC" << std::endl;

  // Make sure that there are score files.
  this->dataPtr->UpdateScoreFiles(this->dataPtr->simTime);
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnBatteryMsg(
    const ignition::msgs::BatteryState &_msg,
    const transport::MessageInfo &_info)
{
  auto simT = this->SimTime();
  ignition::msgs::Time localSimTime(simT);
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
        this->LogEvent("dead_battery", name);
      }
    }
  }
}

//////////////////////////////////////////////////
void GameLogicPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (!this->dataPtr->started)
    return;

  // check if any robots moved outside of geofence
  // give time penalties or disable robots if they exceeded
  // boundaries
  this->dataPtr->CheckRobotsInGeofenceBoundary(_ecm);

  // disable objects dropped into the ocean
  this->dataPtr->DisableDroppedObjects(_ecm);
}

//////////////////////////////////////////////////
void GameLogicPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Store sim time
  int64_t s, ns;
  std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->simTimeMutex);
    this->dataPtr->simTime.set_sec(s);
    this->dataPtr->simTime.set_nsec(ns);
  }

  // Capture the names of the robots. We only do this until the team
  // triggers the start signal.
  if (!this->dataPtr->started)
  {
    this->dataPtr->EnumerateCompetitorPlatforms(_ecm);

    // Start automatically if setup time has elapsed.
    if (this->dataPtr->simTime.sec() >= this->dataPtr->setupTimeSec)
    {
      this->dataPtr->Start(this->dataPtr->simTime);
    }
    else
    {
      // Start automatically if a robot moves for more than x meters
      // from initial position
      for (auto &it : this->dataPtr->robots)
      {
        Entity robotEnt = it.first;
        math::Vector3d robotInitPos = it.second.initialPos;

        it.second.inStartingArea = this->dataPtr->CheckEntityInBoundary(_ecm,
          robotEnt, this->dataPtr->startingBoundary);

        auto poseComp = _ecm.Component<gazebo::components::Pose>(robotEnt);
        if (poseComp)
        {
          double distance = poseComp->Data().Pos().Distance(robotInitPos);
          if (distance > 5.0)
          {
            this->dataPtr->Log(this->dataPtr->simTime)
                << "Robot moved 5 meters from starting location." << std::endl;
            this->dataPtr->Start(this->dataPtr->simTime);
          }
        }
      }
    }
  }

  if (this->dataPtr->started && !this->dataPtr->audited)
  {
    auto valid = this->dataPtr->AuditCompetitorConfiguration(_ecm);

    if (!valid)
    {
      this->dataPtr->Log(this->dataPtr->simTime)
        << "Competitor Configuration Invalid." << std::endl;
    }
    else
    {
      this->dataPtr->Log(this->dataPtr->simTime)
        << "Competitor Configuration Valid." << std::endl;
    }
    this->dataPtr->audited = true;
  }

  // find the sensor associated with the input stream
  // used for validating target reports
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->streamMutex);
    if (!this->dataPtr->targetStreamTopic.empty())
    {
      for (auto sensorEntity : this->dataPtr->cameraSensors)
      {
        std::string topic = scopedName(sensorEntity, _ecm);
        if (this->dataPtr->targetStreamTopic.find(topic) != std::string::npos)
        {
          this->dataPtr->targetStreamSensorEntity = sensorEntity;
          this->dataPtr->targetStreamSensorEntityChanged = true;
        }
      }
    }
    this->dataPtr->targetStreamTopic.clear();
  }

  std::string phaseStr = this->dataPtr->Phase();

  // validate target reports - inspection task
  if (phaseStr == kPhaseStarted ||
      phaseStr == kPhaseVesselIdSuccess ||
      phaseStr == kPhaseSmallObjectIdSuccess ||
      // if there are more than 1 target vessels, we will starting validating
      // target reports again.
      phaseStr == kPhaseLargeObjectRetrieveSuccess)
  {
    // validate target reports
    this->dataPtr->ValidateTargetReports();
  }

  // validate target object retrieval - intervention task
  if (phaseStr == kPhaseLargeObjectIdSuccess ||
      phaseStr == kPhaseSmallObjectRetrieveSuccess)
  {
    this->dataPtr->ValidateTargetObjectRetrieval();
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
    ignition::msgs::StringMsg phaseMsg;
    std::string p = this->dataPtr->Phase();
    phaseMsg.set_data(p);
    if (p == kPhaseSetup)
    {
      competitionClockMsg.mutable_sim()->set_sec(
          this->dataPtr->setupTimeSec - this->dataPtr->simTime.sec());
    }
    else
    {
      auto secondsRemaining = std::chrono::duration_cast<std::chrono::seconds>(
          remainingCompetitionTime);
      competitionClockMsg.mutable_sim()->set_sec(secondsRemaining.count());
    }

    this->dataPtr->competitionClockPub.Publish(competitionClockMsg);
    this->dataPtr->competitionPhasePub.Publish(phaseMsg);
    this->dataPtr->lastStatusPubTime = currentTime;
  }

  // Periodically update the score file.
  if (!this->dataPtr->finished && currentTime -
      this->dataPtr->lastUpdateScoresTime > std::chrono::seconds(1))
  {
    this->dataPtr->UpdateScoreFiles(this->dataPtr->simTime);
  }

  if (this->dataPtr->finished)
  {
    if (this->dataPtr->exitOnFinish &&
        (currentTime - this->dataPtr->finishTime) > std::chrono::seconds(10))
    {
      ignmsg << "Stopping simulation." << std::endl;
      this->dataPtr->eventManager->Emit<events::Stop>();
    }
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::CheckRobotsInGeofenceBoundary(
    EntityComponentManager &_ecm)
{
  // check if robots are inside the competition boundary
  for (auto &[robotEnt, robot]: this->robots)
  {
    if (robot.isDisabled)
    {
      continue;
    }

    // update list of robot and whether or not it is inside boundary
    bool wasInBounds = robot.inCompetitionBoundary;

    // check if it exceeded the outer / hard boundary
    // if so, disable the robot
    if (!wasInBounds && !this->CheckEntityInBoundary(_ecm,
          robotEnt, this->geofenceBoundaryOuter))
    {
      this->MakeStatic(robotEnt, _ecm);
      robot.isDisabled = true;
      this->LogEvent("exceed_boundary_outer", robot.robotName);
      continue;
    }

    // If robot was outside of boundary, allow some buffer distance before
    // counting it as returning to the inside boundary.
    // This is so that we don't immediately trigger another exceed_boundary
    // event if the robot oscillates slightly at the boundary line,
    // e.g. USV motion due to waves
    auto boundary = wasInBounds ? this->geofenceBoundary :
        this->geofenceBoundaryInner;

    bool isInBounds = this->CheckEntityInBoundary(_ecm,
        robotEnt, boundary);

    if (isInBounds != wasInBounds)
    {
      // robot moved outside the boundary!
      if (!isInBounds)
      {
        this->geofenceBoundaryPenaltyCount++;
        if (this->geofenceBoundaryPenaltyCount == 1)
        {
          this->timePenalty +=
              this->kTimePenalties.at(PenaltyType::BOUNDARY_1);
          this->LogEvent("exceed_boundary_1", robot.robotName);
          this->UpdateScoreFiles(this->simTime);
        }
        else if (this->geofenceBoundaryPenaltyCount >= 2)
        {
          this->timePenalty =
              this->kTimePenalties.at(PenaltyType::BOUNDARY_2);
          this->LogEvent("exceed_boundary_2", robot.robotName);
          // terminate run
          this->Finish(this->simTime);
        }
      }
      robot.inCompetitionBoundary = isInBounds;
    }
  }
}

//////////////////////////////////////////////////
bool GameLogicPluginPrivate::MakeStatic(Entity _entity,
    EntityComponentManager &_ecm)
{
  // make entity static by spawning a static model and attaching the
  // entity to the static model
  // todo(anyone) Add a feature in ign-physics to support making a model
  // static
  if (this->staticModelToSpawn.LinkCount() == 0u)
  {
    sdf::ElementPtr staticModelSDF(new sdf::Element);
    sdf::initFile("model.sdf", staticModelSDF);
    staticModelSDF->GetAttribute("name")->Set("static_model");
    staticModelSDF->GetElement("static")->Set(true);
    sdf::ElementPtr linkElem = staticModelSDF->AddElement("link");
    linkElem->GetAttribute("name")->Set("static_link");
    this->staticModelToSpawn.Load(staticModelSDF);
  }

  auto poseComp = _ecm.Component<gazebo::components::Pose>(_entity);
  if (!poseComp)
    return false;
  math::Pose3d p = poseComp->Data();
  this->staticModelToSpawn.SetRawPose(p);

  auto nameComp = _ecm.Component<gazebo::components::Name>(_entity);
  this->staticModelToSpawn.SetName(nameComp->Data() + "__static__");

  Entity staticEntity = this->creator->CreateEntities(&staticModelToSpawn);
  this->creator->SetParent(staticEntity, this->worldEntity);

  Entity parentLinkEntity = _ecm.EntityByComponents(
      gazebo::components::Link(),
      gazebo::components::ParentEntity(staticEntity),
      gazebo::components::Name("static_link"));

  if (parentLinkEntity == kNullEntity)
    return false;

  Entity childLinkEntity = _ecm.EntityByComponents(
      gazebo::components::CanonicalLink(),
      gazebo::components::ParentEntity(_entity));

  if (childLinkEntity == kNullEntity)
    return false;

  Entity detachableJointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(detachableJointEntity,
      gazebo::components::DetachableJoint(
      {parentLinkEntity, childLinkEntity, "fixed"}));

  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::CheckEntityInBoundary(
    const EntityComponentManager &_ecm, Entity _entity,
    const math::AxisAlignedBox &_boundary)
{
  auto poseComp = _ecm.Component<gazebo::components::Pose>(_entity);
  if (!poseComp)
  {
    ignerr << "Pose component not found for Entity: " << _entity
           << std::endl;
    return false;
  }

  // this should be world pose since it is a top level model
  math::Pose3d pose = poseComp->Data();
  return _boundary.Contains(pose.Pos());
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishScore()
{
  transport::Node::Publisher scorePub =
    this->node.Advertise<ignition::msgs::Float>("/mbzirc/score");
  ignition::msgs::Float msg;

  while (!this->finished)
  {
    {
      std::lock_guard<std::mutex> lock(this->scoreMutex);
      msg.set_data(this->totalScore);
    }

    scorePub.Publish(msg);
    IGN_SLEEP_S(1);
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnFinishCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  auto simT = this->SimTime();
  ignition::msgs::Time localSimTime(simT);
  if (_req.data() && !this->finished)
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
  auto simT = this->SimTime();
  ignition::msgs::Time localSimTime(simT);
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
    this->LogEvent("started");
    this->SetPhase(kPhaseStarted);
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

    double score = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->scoreMutex);
      score = this->totalScore;
    }

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
    this->Log(_simTime) << "finished_score " << score << std::endl;
    this->Log(_simTime) << "time_penalty " << this->timePenalty << std::endl;
    this->logStream.flush();

    this->LogEvent("finished");
    this->SetPhase(kPhaseFinished);
  }

  this->finishTime = currTime;
  this->finished = true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnReportTargets(
    const ignition::msgs::StringMsg_V &_req,
    ignition::msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->reportMutex);
  this->reports.push_back(_req);
  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::OnDetectObjectPlacement(
    const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->reportMutex);
  this->objectPlacements.push_back(_msg);
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::OnDetectObjectDropped(
    const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->reportMutex);
  this->objectsDropped.push_back(_msg);
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnTargetStreamStart(
    const ignition::msgs::StringMsg_V &_req,
    ignition::msgs::Boolean &_res)
{
  if (_req.data_size() < 1)
  {
    ignwarn << "Target stream start request: Missing arguments " << std::endl;
    this->LogEvent("stream_start_request", "missing_arg");
    _res.set_data(false);
    return true;
  }

  std::string vehicleName;
  std::string vehicleSensor;

  if (_req.data_size() == 1)
  {
    std::string sensorFrame = _req.data(0);
    if (sensorFrame.empty())
    {
      ignwarn << "Target stream start request: Empty sensor frame"
              << std::endl;
      this->LogEvent("stream_start_request", "empty_sensor_frame");
      _res.set_data(false);
      return true;
    }
    std::vector<std::string> tokens = common::split(sensorFrame, "/");
    if (tokens.size() >= 2)
    {
      vehicleName =  tokens[0];
      vehicleSensor = tokens[1];
    }
  }
  else
  {
    vehicleName = _req.data(0);
    vehicleSensor = _req.data(1);
  }

  if (vehicleName.empty() || vehicleSensor.empty())
  {
    ignwarn << "Target stream start request: Empty vehicle name or sensor"
            << std::endl;
    this->LogEvent("stream_start_request", "empty_vehicle_or_sensor");
    _res.set_data(false);
    return true;
  }

  std::string vehicleTopic;

  // verify that there is a image topic associated with the specified robot
  // and sensor
  std::vector<std::string> allTopics;
  this->node.TopicList(allTopics);
  for (auto topic : allTopics)
  {
    std::vector<transport::MessagePublisher> publishers;
    this->node.TopicInfo(topic, publishers);
    for (auto pub : publishers)
    {
      if (pub.MsgTypeName() == "ignition.msgs.Image")
      {
        if (topic.find("/model/" + vehicleName + "/") != std::string::npos &&
            topic.find("/" + vehicleSensor +"/") != std::string::npos)
        {
          vehicleTopic = topic;
          break;
        }
      }
    }
  }

  if (!vehicleTopic.empty())
  {
    _res.set_data(true);
    std::lock_guard<std::mutex> lock(this->streamMutex);
    this->targetStreamTopic = vehicleTopic;

    ignmsg << "Target stream start request: success" << std::endl;
    this->LogEvent("stream_start_request", "success");
    this->PublishStreamStatus("stream_started");
  }
  else
  {
    ignwarn << "Target stream start request: Unable to find image topic for "
            << "vehicle: " << vehicleName << ", and sensor: " << vehicleSensor
            << std::endl;
    _res.set_data(false);
    this->LogEvent("stream_start_request", "no_image_topic_found");
    this->PublishStreamStatus("stream_start_failed");

    std::lock_guard<std::mutex> lock(this->streamMutex);
    this->targetStreamTopic.clear();
  }
  this->targetStreamSensorEntity = kNullEntity;
  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnTargetStreamStop(
    const ignition::msgs::Empty &_req,
    ignition::msgs::Boolean &_res)
{
  _res.set_data(true);

  std::lock_guard<std::mutex> lock(this->streamMutex);
  this->targetStreamTopic.clear();

  ignmsg << "Target stream stop request: success" << std::endl;
  this->LogEvent("stream_stopped", this->targetStreamTopic);
  this->PublishStreamStatus("stream_stopped");

  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnTargetStreamReport(
    const ignition::msgs::StringMsg_V &_req,
    ignition::msgs::Boolean &_res)
{
  if (!this->started || this->finished)
  {
    ignwarn << "Unable to report target in stream. Run not active"
            << std::endl;
    this->LogEvent("target_reported_in_stream", "run_not_active");
    _res.set_data(false);
    this->PublishStreamStatus("target_reported_run_not_active");
    return true;
  }

  std::string type = _req.data(0);
  unsigned int x = std::stoul(_req.data(1));
  unsigned int y = std::stoul(_req.data(2));

  TargetInStream tis;
  tis.type = type;
  tis.x = x;
  tis.y = y;

  std::lock_guard<std::mutex> lock(this->streamMutex);
  // there can only be one target report at a time
  this->targetInStreamReport = tis;

  // Set up the render connection so we can validate target in the rendering
  // thread
  if (!this->postRenderConn)
  {
    this->postRenderConn =
        this->eventManager->Connect<events::PostRender>(
        std::bind(&GameLogicPluginPrivate::OnPostRender, this));
  }

  std::string eventData = type + "::" + std::to_string(x) + "::" +
                          std::to_string(y);
  ignmsg << "Target report in stream: " << type << " " << x << " " << y
         << std::endl;
  this->LogEvent("target_reported_in_stream", eventData);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnSkipToPhase(
    const ignition::msgs::StringMsg &_req,
    ignition::msgs::Boolean &_res)
{
  std::string p = _req.data();
  if (p != kPhaseSetup)
  {
    auto simT = this->SimTime();
    this->Start(simT);
  }
  else
  {
    this->started = false;
  }
  this->SetPhase(p);
  ignmsg << "Skipping to phase: " << p << std::endl;

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PauseVesselTrajectory(const std::string &_vessel)
{
  std::string topic = "/model/" + _vessel + "/trajectory_follower/pause";
  topic = transport::TopicUtils::AsValidTopic(topic);
  transport::Node::Publisher pub =
      this->node.Advertise<ignition::msgs::Boolean>(topic);
  ignition::msgs::Boolean msg;
  msg.set_data(true);
  pub.Publish(msg);
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::DetachTargetObjects(const std::string &_vessel)
{
  std::string topic = "/" + _vessel + "/detach";
  topic = transport::TopicUtils::AsValidTopic(topic);
  transport::Node::Publisher pub =
      this->node.Advertise<ignition::msgs::Empty>(topic);
  ignition::msgs::Empty msg;
  pub.Publish(msg);
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::ValidateTargetReports()
{
  std::lock_guard<std::mutex> lock(this->reportMutex);
  for (auto &req : this->reports)
  {
    if (req.data_size() <= 0)
    {
      this->LogEvent(kTargetReported, "empty_report");
      continue;
    }

    std::string vessel = req.data(0);
    if (vessel.empty())
    {
      this->LogEvent(kTargetReported, "target_vessel_missing");
      continue;
    }

    auto simT = this->SimTime();

    auto it = this->targets.find(vessel);
    if (it != this->targets.end())
    {
      auto target = it->second;

      // target has not been report - valid report
      if (!target.vesselReported)
      {
        target.vesselReported = true;
        this->targets[vessel] = target;
        this->LogEvent(kTargetReported, "vessel_id_success");
        this->currentTargetVessel = vessel;
        this->SetPhase(kPhaseVesselIdSuccess);
        this->PublishStreamStatus("vessel_id_success");

        ignmsg << "Target vessel identified: " << vessel << ". "
               << "Pausing trajectory following and detaching target objects. "
               << std::endl;
        this->PauseVesselTrajectory(vessel);
        this->DetachTargetObjects(vessel);

        continue;
      }
      // target has already been reported
      // team is reporting objects
      else if (req.data_size() > 1)
      {
        // small object target
        std::string smallObj = req.data(1);
        if (!smallObj.empty())
        {
          auto sIt = target.smallObjects.find(smallObj);
          bool validObj = sIt != target.smallObjects.end();
          if (validObj)
          {
            auto reportIt = target.smallObjectsReported.find(smallObj);
            if (reportIt == target.smallObjectsReported.end())
            {
              target.smallObjectsReported.insert(smallObj);
              this->targets[vessel] = target;
              this->LogEvent(kTargetReported, kPhaseSmallObjectIdSuccess);
              this->SetPhase(kPhaseSmallObjectIdSuccess);
              this->PublishStreamStatus(kPhaseSmallObjectIdSuccess);
              continue;
            }
            else
            {
              this->LogEvent(kTargetReported, "small_object_id_duplicate");
            }
          }
          else
          {
            // add penalty for incorrectly identifying small object
            std::string logData = "small_object_id_failure";
            this->PublishStreamStatus(logData);
            unsigned int count = 0u;
            auto it = this->smallObjectIdPenaltyCount.find(vessel);
            if (it != this->smallObjectIdPenaltyCount.end())
              count = it->second;

            this->smallObjectIdPenaltyCount[vessel] = ++count;
            if (count == 1u)
            {
              this->timePenalty +=
                  this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_ID_1);
              this->UpdateScoreFiles(simT);
              logData += "_1";
              this->LogEvent(kTargetReported, logData);
            }
            else if (count == 2u)
            {
              this->timePenalty +=
                  this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_ID_2);
              this->UpdateScoreFiles(simT);
              logData += "_2";
              this->LogEvent(kTargetReported, logData);
            }
            else
            {
              this->timePenalty +=
                  this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_ID_3);
              logData += "_3";
              this->LogEvent(kTargetReported, logData);
              // terminate run
              this->Finish(simT);
              break;
            }
          }
        }
        else if (req.data_size() > 2)
        {
          // large object target
          std::string largeObj = req.data(2);
          if (!largeObj.empty())
          {
            auto sIt = target.largeObjects.find(largeObj);
            bool validObj = sIt != target.largeObjects.end();
            if (validObj)
            {
              auto reportIt = target.largeObjectsReported.find(largeObj);
              if (reportIt == target.largeObjectsReported.end())
              {
                target.largeObjectsReported.insert(largeObj);
                this->targets[vessel] = target;
                this->LogEvent(kTargetReported, kPhaseLargeObjectIdSuccess);
                this->SetPhase(kPhaseLargeObjectIdSuccess);
                this->PublishStreamStatus(kPhaseLargeObjectIdSuccess);
                continue;
              }
              else
              {
                this->LogEvent(kTargetReported, "large_object_id_duplicate");
              }
            }
            else
            {
              // add penalty for incorrectly identifying large object
              std::string logData = "large_object_id_failure";
              this->PublishStreamStatus(logData);
              unsigned int count = 0u;
              auto it = this->largeObjectIdPenaltyCount.find(vessel);
              if (it != this->largeObjectIdPenaltyCount.end())
                count = it->second;

              this->largeObjectIdPenaltyCount[vessel] = ++count;
              if (count == 1u)
              {
                this->timePenalty +=
                    this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_ID_1);
                this->UpdateScoreFiles(simT);
                this->LogEvent(kTargetReported, logData + "_1");
              }
              else if (count == 2u)
              {
                this->timePenalty +=
                    this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_ID_2);
                this->UpdateScoreFiles(simT);
                this->LogEvent(kTargetReported, logData + "_2");
              }
              else
              {
                this->timePenalty +=
                    this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_ID_3);
                this->LogEvent(kTargetReported, logData + "_3");
                // terminate run
                this->Finish(simT);
                break;
              }
            }
          }
        }
      }
      else
      {
        this->LogEvent(kTargetReported, "vessel_id_duplicate");
      }
    }
    else
    {
      // add penalty for incorrectly identifying vessel
      std::string logData = "vessel_id_failure";
      this->PublishStreamStatus(logData);
      this->vesselPenaltyCount++;
      if (this->vesselPenaltyCount == 1u)
      {
        this->timePenalty +=
            this->kTimePenalties.at(PenaltyType::TARGET_VESSEL_ID_1);
        this->UpdateScoreFiles(simT);
        this->LogEvent(kTargetReported, logData + "_1");
      }
      else if (this->vesselPenaltyCount == 2u)
      {
        this->timePenalty +=
            this->kTimePenalties.at(PenaltyType::TARGET_VESSEL_ID_2);
        this->UpdateScoreFiles(simT);
        this->LogEvent(kTargetReported, logData + "_2");
      }
      else
      {
        this->timePenalty +=
            this->kTimePenalties.at(PenaltyType::TARGET_VESSEL_ID_3);
        this->LogEvent(kTargetReported, logData + "_3");
        // terminate run
        this->Finish(simT);
        break;
      }
    }
  }
  this->reports.clear();
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::ValidateTargetObjectRetrieval()
{
  // check if any target objects are dropped into the ocean
  // if so, give penalty
  std::lock_guard<std::mutex> lock(this->reportMutex);
  std::string phaseStr = this->Phase();
  auto simT = this->SimTime();
  // std::string vessel = this->currentTargetVessel;
  // auto &target = this->targets[vessel];
  for (const auto &msg: this->objectsDropped)
  {
    std::string objName = msg.name();
    // ignore tmp model created to make dropped object static
    if (objName.find("_static_") != std::string::npos)
      continue;
    // check of object entered region that counts as "dropped"
    std::string v = msg.header().data(1).value(0);
    bool enteredRegion = std::stoi(v);
    if (!enteredRegion)
      continue;

    // check if small object is dropped
    for (const auto &targetIt : this->targets)
    {
      auto &target = targetIt.second;
      std::string vessel = targetIt.first;
      for (const auto &smallObj : target.smallObjects)
      {
        if (objName.find(smallObj) != std::string::npos)
        {
          unsigned int count = 0u;
          auto it = this->smallObjectRetrievePenaltyCount.find(vessel);
          if (it != this->smallObjectRetrievePenaltyCount.end())
            count = it->second;

          this->smallObjectRetrievePenaltyCount[vessel] = ++count;
          std::string logData = "small_object_retrieve_failure";
          if (count == 1u)
          {
            this->timePenalty +=
                this->kTimePenalties.at(PenaltyType::SMALL_OBJECT_RETRIEVE_1);
            this->UpdateScoreFiles(simT);
            logData += "_1";
            this->LogEvent(kTargetRetrieval, logData);
          }
          else if (count == 2u)
          {
            this->timePenalty +=
                this->kTimePenalties.at(PenaltyType::SMALL_OBJECT_RETRIEVE_2);
            logData += "_2";
            this->LogEvent(kTargetRetrieval, logData);
            // terminate run
            this->Finish(simT);
          }
          break;
        }
      }
    }
    // check if large object is dropped
    for (const auto &targetIt : this->targets)
    {
      auto &target = targetIt.second;
      std::string vessel = targetIt.first;
      for (const auto &largeObj : target.largeObjects)
      {
        if (objName.find(largeObj) != std::string::npos)
        {
          unsigned int count = 0u;
          auto it = this->largeObjectRetrievePenaltyCount.find(vessel);
          if (it != this->largeObjectRetrievePenaltyCount.end())
            count = it->second;

          this->largeObjectRetrievePenaltyCount[vessel] = ++count;
          std::string logData = "large_object_retrieve_failure";
          if (count == 1u)
          {
            this->timePenalty +=
                this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_RETRIEVE_1);
            this->UpdateScoreFiles(simT);
            logData += "_1";
            this->LogEvent(kTargetRetrieval, logData);
          }
          else if (count == 2u)
          {
            this->timePenalty +=
                this->kTimePenalties.at(PenaltyType::LARGE_OBJECT_RETRIEVE_2);
            logData += "_2";
            this->LogEvent(kTargetRetrieval, logData);
            // terminate run
            this->Finish(simT);
          }
          break;
        }
      }
    }

    // make the object static
    this->objectsToDisable.insert(objName);
  }
  this->objectsDropped.clear();

  // iterate over object placements to see which object has been placed
  for (const auto &msg: this->objectPlacements)
  {
    std::string objName = msg.name();
    std::string v = msg.header().data(1).value(0);
    bool enteredRegion = std::stoi(v);
    if (!enteredRegion)
      continue;
    // phase is currently large_object_id_success, that means
    // the inspection phase is done and we are now in the intervention phase.
    // The next task is grabbing the small target object
    if (phaseStr == kPhaseLargeObjectIdSuccess)
    {
      for (auto &targetIt : this->targets)
      {
        auto &target = targetIt.second;
        for (auto &smallObj : target.smallObjects)
        {
          if (objName.find(smallObj) != std::string::npos &&
              target.smallObjectsRetrieved.find(objName) ==
              target.smallObjectsRetrieved.end())
          {
            this->LogEvent(kTargetRetrieval, kPhaseSmallObjectRetrieveSuccess);
            this->SetPhase(kPhaseSmallObjectRetrieveSuccess);
            target.smallObjectsRetrieved.insert(objName);
            break;
          }
        }
      }
    }
    // phase is currently small_object_id_success, that means
    // the next task is grabbing the large target object
    else if (phaseStr == kPhaseSmallObjectRetrieveSuccess)
    {
      for (auto &targetIt : this->targets)
      {
        auto &target = targetIt.second;
        for (auto &largeObj : target.largeObjects)
        {
          if (objName.find(largeObj) != std::string::npos &&
              target.largeObjectsRetrieved.find(objName) ==
              target.largeObjectsRetrieved.end())
          {
            target.largeObjectsRetrieved.insert(objName);
            this->LogEvent(kTargetRetrieval, kPhaseLargeObjectRetrieveSuccess);
            this->SetPhase(kPhaseLargeObjectRetrieveSuccess);
            this->CheckTaskCompletion();
          }
        }
      }
    }
  }
  this->objectPlacements.clear();
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::DisableDroppedObjects(
    EntityComponentManager &_ecm)
{
  for (const auto &objName : this->objectsToDisable)
  {
    auto entity = _ecm.EntityByComponents(gazebo::components::Name(objName));
    if (entity != kNullEntity)
      this->MakeStatic(entity, _ecm);
  }
  this->objectsToDisable.clear();
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::EnumerateCompetitorPlatforms(
  const EntityComponentManager &_ecm)
{
  _ecm.Each<gazebo::components::Sensor,
            gazebo::components::ParentEntity>(
    [&](const gazebo::Entity &_entity,
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
        gazebo::Entity entity = model->Data();

        if (!this->robots.count(entity))
        {
          PlatformInfo info;
          info.modelEntity = entity;
          info.robotName = _ecm.Component<gazebo::components::Name>(entity)->Data();
          info.initialPos = _ecm.Component<gazebo::components::Pose>(entity)->Data().Pos();
          this->robots[entity] = info;
          this->EnumerateCompetitorSensors(entity, _ecm);
          igndbg << "New Competitor Platform: " << info.robotName << " with " << info.sensors.size() << "sensors\n";


          // Subscribe to battery state in order to log battery events.
          std::string batteryTopic = std::string("/model/") +
            info.robotName + "/battery/linear_battery/state";
          this->node.Subscribe(batteryTopic,
              &GameLogicPluginPrivate::OnBatteryMsg, this);
        }
        // store camera / rgbd camera sensor
        // later used for target confirmation in image stream
        auto camComp = _ecm.Component<gazebo::components::Camera>(_entity);
        if (camComp)
        {
          this->cameraSensors.insert(_entity);
        }
        auto rgbdComp = _ecm.Component<gazebo::components::RgbdCamera>(_entity);
        if (rgbdComp)
        {
          this->cameraSensors.insert(_entity);
        }
      }
      return true;
    }
  );
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::EnumerateCompetitorSensors(Entity _entity, const EntityComponentManager &_ecm)
{
  auto info = this->robots[_entity];

  _ecm.Each<gazebo::components::Sensor,
            gazebo::components::ParentEntity>(
    [&](const gazebo::Entity &_sensorEntity,
        const gazebo::components::Sensor *,
        const gazebo::components::ParentEntity *_parent) -> bool
    {
      std::string slot_name;
      auto parent = _ecm.Component<gazebo::components::ParentEntity>(
          _parent->Data());
      auto model = parent;

      // find top level model
      while (parent && _ecm.Component<gazebo::components::Model>(
             parent->Data()))
      {
        auto name = _ecm.Component<gazebo::components::Name>(parent->Data());
        if (name->Data().find("sensor_") != std::string::npos)
        {
          slot_name = name->Data();
        }

        model = parent;
        parent = _ecm.Component<gazebo::components::ParentEntity>(
            parent->Data());
      }

      // Don't count sensors not in slots
      if (model && !slot_name.empty())
      {
        gazebo::Entity modelEntity = model->Data();

        // The sensor belongs to this platform
        if (modelEntity == _entity)
        {
          auto sensorInfo = SensorInfo();
          sensorInfo.sensorEntity = _sensorEntity;
          sensorInfo.slotName = slot_name;

          if (auto camera = _ecm.Component<gazebo::components::Camera>(_sensorEntity))
          {
            sensorInfo.sensorType = "camera";
          }
          else if (_ecm.Component<gazebo::components::RgbdCamera>(_sensorEntity))
          {
            sensorInfo.sensorType = "rgbd_camera";
          }
          else if (_ecm.Component<gazebo::components::GpuLidar>(_sensorEntity))
          {
            sensorInfo.sensorType = "gpu_lidar";
          }
          info.sensors.push_back(sensorInfo);
        }
      }
      return true;
    });
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::AuditCompetitorConfiguration(const EntityComponentManager &_ecm)
{
  bool valid = true;
  int total_usvs = 0;
  int total_sensors = 0;

  std::stringstream ss;

  // All initial positions need to be in the starting area
  for (const auto &[robotEnt, robot]: this->robots)
  {
    if (!this->startingBoundary.Contains(robot.initialPos))
    {
      valid = false;
      ss << "[" << robot.robotName << "]";
      ss << " Initial Position (" << robot.initialPos << ") is not in the starting area " << this->startingBoundary.Min() << " " << this->startingBoundary.Max() << "\n";
    }
    total_sensors += robot.sensors.size();
    if (_ecm.EntityHasComponentType(robotEnt, mbzirc::components::Usv().TypeId()))
    {
      total_usvs++;
    }
  }

  if (total_sensors > this->maximumSensors)
  {
    ss << "Total sensors (" << total_sensors << ") exceeds maximum allowed (" << this->maximumSensors << ")\n";
    valid = false;
  }

  if (total_usvs > this->maximumUsvs)
  {
    ss << "Total usvs (" << total_usvs << ") exceeds maximum allowed (" << this->maximumUsvs<< ")\n";
    valid = false;
  }

  if (!valid)
  {
    ignerr << "Invalid configuration detected: \n" << ss.str();
  }

  return valid;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::CheckTaskCompletion()
{
  bool completed = true;
  for (const auto &targetIt : this->targets)
  {
    if (targetIt.second.largeObjectsRetrieved.empty())
    {
      completed = false;
    }
  }

  if (completed)
  {
    auto simT = this->SimTime();
    this->Finish(simT);
    ignmsg << "All tasks have been completed! " << std::endl;
    ignmsg << "Score: " << this->totalScore << std::endl;
  }
}

/////////////////////////////////////////////////
ignition::msgs::Time GameLogicPluginPrivate::SimTime()
{
  std::lock_guard<std::mutex> lock(this->simTimeMutex);
  return this->simTime;
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
  summary << "model_count: " << this->robots.size() << std::endl;
  summary << "time_penalty: " << this->timePenalty << std::endl;
  summary << "phase: " << this->Phase() << std::endl;
  summary.flush();

  // Output a score file with just the final score
  std::ofstream scoreFile(this->logPath + "/score.yml", std::ios::out);
  {
    std::lock_guard<std::mutex> lock(this->scoreMutex);
    this->totalScore = (math::equal(this->timePenalty, ignition::math::MAX_I32)) ?
        ignition::math::MAX_I32: simElapsed + this->timePenalty;

    scoreFile << this->totalScore << std::endl;
  }
  scoreFile.flush();

  this->lastUpdateScoresTime = currTime;
  return currTime;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::LogEvent(const std::string &_type,
    const std::string &_data)
{
  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;
  std::chrono::steady_clock::time_point currTime =
    std::chrono::steady_clock::now();

  auto simT = this->SimTime();

  if (this->started)
  {
    realElapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currTime - this->startTime).count();
    simElapsed = simT.sec() - this->startSimTime.sec();
  }

  std::lock_guard<std::mutex> lock(this->eventCounterMutex);
  std::lock_guard<std::mutex> lock2(this->scoreMutex);
  std::ostringstream stream;
  stream
    << "- event:\n"
    << "  id: " << this->eventCounter << "\n"
    << "  type: " << _type << "\n"
    << "  time_sec: " << simT.sec() << "\n"
    << "  elapsed_real_time: " << realElapsed << "\n"
    << "  elapsed_sim_time: " << simElapsed << "\n"
    << "  total_score: " << this->totalScore << std::endl;
  if (!_data.empty())
    stream << "  data: " << _data << std::endl;

  this->eventStream << stream.str();
  this->eventStream.flush();
  this->eventCounter++;

  this->Log(simT) << "Logged Event:\n" << stream.str() << std::endl;
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPluginPrivate::Log(
    const ignition::msgs::Time &_simTime)
{
  this->logStream << _simTime.sec()
                  << " " << _simTime.nsec() << " ";
  return this->logStream;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::OnPostRender()
{
  // get scene
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  // return if scene not ready or no sensors available.
  if (!this->scene->IsInitialized() ||
      this->scene->SensorCount() == 0)
  {
    return;
  }

  // get and store visuals of targets
  if (this->targetVesselVisuals.empty() ||
      this->targetSmallObjectVisuals.empty() ||
      this->targetLargeObjectVisuals.empty())
  {
    for (const auto &it : this->targets)
    {
      rendering::VisualPtr vesselVisual =
          this->scene->VisualByName(it.first);
      if (!vesselVisual)
      {
        ignerr << "Unable to find visual for vessel: " << it.first << std::endl;
      }
      else
      {
        this->targetVesselVisuals.insert(vesselVisual);
      }


      for (const auto &obj : it.second.smallObjects)
      {
        rendering::VisualPtr objVisual =
            this->scene->VisualByName(obj);
        if (!objVisual)
        {
          ignerr << "Unable to find visual for small object: " << it.first
                 << std::endl;
        }
        else
        {
          this->targetSmallObjectVisuals[it.first].insert(objVisual);
        }
      }
      for (const auto &obj : it.second.largeObjects)
      {
        rendering::VisualPtr objVisual =
            this->scene->VisualByName(obj);
        if (!objVisual)
        {
          ignerr << "Unable to find visual for large object: " << it.first
                 << std::endl;
        }
        else
        {
          this->targetLargeObjectVisuals[it.first].insert(objVisual);
        }
      }
    }
  }

  // get camera currently streaming video
  std::lock_guard<std::mutex> lock(this->streamMutex);
  if (this->targetStreamSensorEntityChanged)
  {
    for (unsigned int i = 0; i < this->scene->SensorCount(); ++i)
    {
      auto sensor = this->scene->SensorByIndex(i);
      if (!sensor)
        continue;

      if (sensor->HasUserData("gazebo-entity"))
      {
        // RenderUtil stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Ignition H
        auto variant = sensor->UserData("gazebo-entity");
        const int *value = std::get_if<int>(&variant);

        if (value && *value == static_cast<int>(this->targetStreamSensorEntity))
        {
          this->camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
          this->targetStreamSensorEntityChanged = false;
          break;
        }
      }
    }
  }
  // reset camera pointer if stream sensor entity is null
  // this could be the stream stopped
  else if (this->camera && this->targetStreamSensorEntity == kNullEntity)
  {
    this->camera.reset();
  }

  // validate target
  if (this->camera && !this->targetInStreamReport.type.empty())
  {
    // save image of target report to log dir
    this->SaveImage(this->targetInStreamReport.type);

    // check if the specified img pos contains the target visual
    // using the FindTargetVisual function below
    // It uses a 2 phase process for identifying target
    // First it checks if target is at the exact img pos
    // If not, it checks to see if any target is nearby with some tol
    ignition::msgs::StringMsg_V req;
    ignition::msgs::Boolean res;
    std::string target;
    // verify vesel
    if (this->targetInStreamReport.type == "vessel")
    {
      for (const auto &v : this->targetVesselVisuals)
      {
        bool found = this->FindTargetVisual(v,
            math::Vector2i(this->targetInStreamReport.x,
            this->targetInStreamReport.y),
            this->targetInStreamReport.type, target);
        if (found)
        {
          break;
        }
      }
      req.add_data(target);
      this->OnReportTargets(req, res);
    }
    // verify small object
    else if (this->targetInStreamReport.type == "small")
    {
      auto it = this->targetSmallObjectVisuals.find(this->currentTargetVessel);
      if (this->currentTargetVessel.empty() ||
          it == this->targetSmallObjectVisuals.end())
      {
        ignerr << "Target vessel has not been identified yet" << std::endl;
        this->targetInStreamReport.type.clear();
        return;
      }

      for (const auto &v : it->second)
      {
        bool found = this->FindTargetVisual(v,
            math::Vector2i(this->targetInStreamReport.x,
            this->targetInStreamReport.y),
            this->targetInStreamReport.type, target);
        if (found)
        {
          break;
        }
      }

      req.add_data(this->currentTargetVessel);
      req.add_data(target);
      this->OnReportTargets(req, res);
    }
    // verify large object
    else if (this->targetInStreamReport.type == "large")
    {
      auto it = this->targetLargeObjectVisuals.find(this->currentTargetVessel);
      if (this->currentTargetVessel.empty() ||
          it == this->targetLargeObjectVisuals.end())
      {
        ignerr << "Target vessel has not been identified yet" << std::endl;
        this->targetInStreamReport.type.clear();
        return;
      }

      for (const auto &v : it->second)
      {
        bool found = this->FindTargetVisual(v,
            math::Vector2i(this->targetInStreamReport.x,
            this->targetInStreamReport.y),
            this->targetInStreamReport.type, target);
        if (found)
        {
          break;
        }
      }

      req.add_data(this->currentTargetVessel);
      req.add_data("");
      req.add_data(target);
      this->OnReportTargets(req, res);
    }

    this->targetInStreamReport.type.clear();
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::SaveImage(const std::string &_type)
{
  // save an image
  rendering::Image image = this->camera->CreateImage();
  this->camera->Copy(image);
  common::Image img;
  unsigned char *data = image.Data<unsigned char>();
  img.SetFromData(data, image.Width(), image.Height(),
      common::Image::RGB_INT8);

  std::string filename;
  int t = 0;
  while (filename.empty() || common::exists(filename))
  {
    filename = ignition::common::joinPaths(this->targetImagesPath,
        _type + "_" + std::to_string(t++) + ".png");
  }
  img.SavePNG(filename);
}


/////////////////////////////////////////////////
bool GameLogicPluginPrivate::FindTargetVisual(
    const rendering::VisualPtr &_targetVis,
    const math::Vector2i &_imagePos, const std::string &_type,
    std::string &_objectAtImgPos) const
{
  // first check if object at specified image pos is the target visual
  std::string target;
  auto vis = this->VisualAt(_imagePos.X(), _imagePos.Y(), _type);
  if (!vis || vis != _targetVis)
  {
    ignmsg << "No valid target at (" << _imagePos << ")." << std::endl;
    ignmsg << "Checking nearby pixels." << std::endl;
    // check if target vessel is in camera view
    math::Vector2i pos;
    if (this->VisualInView(_targetVis, pos))
    {
      ignmsg << "Target is in view: " << _targetVis->Name() << " at "
             << pos << std::endl;
      // check image pos of target is within tolernace
      auto diff = pos - _imagePos;

      // todo(anyone) make tol a function of distance from camera to
      // target as well?
      double tol = this->kTargetVesselInImageTol;
      if (_type != "vessel")
        tol = this->kTargetObjInImageTol;

      double pxTol = this->camera->ImageWidth() * tol;
      if (diff.Length() < pxTol)
      {
        _objectAtImgPos = _targetVis->Name();
        ignmsg << "  Found target: " << _targetVis->Name() << " "
               << "within the allowed tolerance. Valid report."
               << std::endl;
        return true;
      }
      else
      {
        ignmsg << "  Target is not within the allowed tolerance: "
               << "diff: " << diff.Length() << ", tol: " << pxTol
               << std::endl;
      }
    }
    else
    {
      ignmsg << "Target is not in view: " << _targetVis->Name() << std::endl;
    }
  }
  else
  {
    ignmsg << "Found target: " << vis->Name() << ". Valid report."
           <<  std::endl;
    _objectAtImgPos = vis->Name();
    return true;
  }

  // set object at img pos even if it is not the target
  if (vis)
    _objectAtImgPos = vis->Name();
  else
    _objectAtImgPos = "none";

  return false;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::VisualInView(
    const rendering::VisualPtr &_visual, math::Vector2i &_imagePos) const
{
  // transfrom visual to camera frame
  auto poseInCameraFrame =
      this->camera->WorldPose().Inverse() * _visual->WorldPose();

  // check if visual is in front of camera and within frustum far clip distance
  if (poseInCameraFrame.X() > 0 &&
      poseInCameraFrame.X() < this->camera->FarClipPlane())
  {
    // check if projected pos of visual is within image width and height
    auto projectedImgPos = this->camera->Project(_visual->WorldPosition());
    if (projectedImgPos.X() >= 0 &&
        projectedImgPos.X() < this->camera->ImageWidth() &&
        projectedImgPos.Y() >= 0 &&
        projectedImgPos.Y() < this->camera->ImageHeight())
    {
      _imagePos = projectedImgPos;
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
rendering::VisualPtr GameLogicPluginPrivate::VisualAt(
    unsigned int _x, unsigned int _y, const std::string &_type) const
{
  math::Vector2i imagePos(_x, _y);

  // hide other target visuals when doing a visual check
  // Why do we do this? For example, when checking a target vessel,
  // we do not want the Camera::VisualAt check below to return the visual
  // of a target object sitting on top of the vessel.
  for (auto &v : this->targetVesselVisuals)
  {
    if (_type == "small" || _type == "large")
      v->SetVisible(false);
  }
  for (auto &it : this->targetSmallObjectVisuals)
  {
    for (auto &v : it.second)
    {
      if (_type == "vessel" || _type == "large")
        v->SetVisible(false);
    }
  }
  for (auto &it : this->targetLargeObjectVisuals)
  {
    for (auto &v : it.second)
    {
      if (_type == "vessel" || _type == "small")
        v->SetVisible(false);
    }
  }

  // visual check - this returns the first visible visual at specified
  // image pos
  rendering::VisualPtr visual = this->camera->VisualAt(imagePos);

  // restore visual visibility after doing a visual check
  for (auto &v : this->targetVesselVisuals)
  {
    v->SetVisible(true);
  }
  for (auto &it : this->targetSmallObjectVisuals)
  {
    for (auto &v : it.second)
      v->SetVisible(true);
  }
  for (auto &it : this->targetLargeObjectVisuals)
  {
    for (auto &v : it.second)
      v->SetVisible(true);
  }

  // check if any visual is found
  // if so, get the top level visual (model) name
  std::string target;
  if (visual)
  {
    auto target = visual;
    while (target->Parent() &&
           target->Parent() != this->scene->RootVisual())
    {
      target = std::dynamic_pointer_cast<rendering::Visual>(
          target->Parent());
    }
    return target;
  }
  return nullptr;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishStreamStatus(const std::string &_status)
{
  ignition::msgs::StringMsg statusMsg;
  statusMsg.set_data(_status);
  this->targetStreamStatusPub.Publish(statusMsg);
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::SetPhase(const std::string &_phase)
{
  std::lock_guard<std::mutex> lock(this->phaseMutex);
  this->phase = _phase;
}

/////////////////////////////////////////////////
std::string GameLogicPluginPrivate::Phase()
{
  std::lock_guard<std::mutex> lock(this->phaseMutex);
  return this->phase;
}
