#ifndef MBZIRCTYPES_HH_
#define MBZIRCTYPES_HH_

#include <ignition/gazebo/Entity.hh>
#include <ignition/math/Vector3.hh>

#include <string>
#include <unordered_set>
#include <vector>

namespace mbzirc
{
constexpr const char* kTargetReported = "target_reported";
constexpr const char* kTargetRetrieval = "target_retrieval";

constexpr const char* kPhaseSetup = "setup";
constexpr const char* kPhaseStarted = "started";
constexpr const char* kPhaseVesselIdSuccess = "vessel_id_success";
constexpr const char* kPhaseSmallObjectIdSuccess = "small_object_id_success";
constexpr const char* kPhaseSmallObjectRetrieveSuccess = "small_object_retrieve_success";
constexpr const char* kPhaseLargeObjectIdSuccess = "large_object_id_success";
constexpr const char* kPhaseLargeObjectRetrieveSuccess = "large_object_retrieve_success";
constexpr const char* kPhaseFinished = "finished";

/// \brief Represents a sensor attached to a competitor platform
struct SensorInfo
{
  /// \brief Entity that the sensor is attached to
  ignition::gazebo::Entity sensorEntity;

  /// \brief Name of the platform slot the sensor is attached to
  std::string slotName;

  /// \brief Sensor type in (camera, rgbd_camera, gpu_lidar)
  std::string sensorType;
};

/// \brief Structure to hold information about competitor platforms
struct PlatformInfo
{
  /// \brief Entity of tAhe platform
  ignition::gazebo::Entity modelEntity;

  /// \brief Name of the platform
  std::string robotName;

  /// \brief Spawn position of the platform
  ignition::math::Vector3d initialPos;

  /// \brief Is the platform inside the competition boundary
  bool inCompetitionBoundary = {true};
      
  /// \brief Is the platform inside the starting area 
  bool inStartingArea = {true};

  /// \brief Is the platform disabled 
  bool isDisabled = {false};

  /// \brief Sensors attached to the platform
  std::vector<SensorInfo> sensors;
};

/// \brief Target vessel, objects, and report status
struct Target
{
  /// \brief Name of target vessel
  std::string vessel;

  /// \brief List of small target objects
  std::unordered_set<std::string> smallObjects;

  /// \brief List of large target objects
  std::unordered_set<std::string> largeObjects;

  /// \brief Indicates if vessel has been reported
  bool vesselReported = false;

  /// \brief Set of small objects that have been reported.
  std::unordered_set<std::string> smallObjectsReported;

  /// \brief Set of large objects that have been reported.
  std::unordered_set<std::string> largeObjectsReported;

  /// \brief Set of small objects that have been retrieved.
  std::unordered_set<std::string> smallObjectsRetrieved;

  /// \brief Set of large objects that have been retrieved.
  std::unordered_set<std::string> largeObjectsRetrieved;
};

/// \brief Information of target in stream
struct TargetInStream
{
  /// \brief Type of target: vessel, small, large
  std::string type;

  /// \brief Image x position
  unsigned int x = 0;

  /// \brief Image y position
  unsigned int y = 0;
};

enum class PenaltyType
{
  /// \brief Failure to ID target vessel
  TARGET_VESSEL_ID_1 = 0,
  TARGET_VESSEL_ID_2 = 1,
  TARGET_VESSEL_ID_3 = 2,

  /// \brief Failure to ID small object
  SMALL_OBJECT_ID_1 = 3,
  SMALL_OBJECT_ID_2 = 4,
  SMALL_OBJECT_ID_3 = 5,

  /// \brief Failure to ID large object
  LARGE_OBJECT_ID_1 = 6,
  LARGE_OBJECT_ID_2 = 7,
  LARGE_OBJECT_ID_3 = 8,

  /// \brief Failure to retrieve / place small object
  SMALL_OBJECT_RETRIEVE_1 = 9,
  SMALL_OBJECT_RETRIEVE_2 = 10,

  /// \brief Failure to retrieve / place large object
  LARGE_OBJECT_RETRIEVE_1 = 11,
  LARGE_OBJECT_RETRIEVE_2 = 12,

  /// \brief Failure to remain in demonstration area boundary
  BOUNDARY_1 = 13,
  BOUNDARY_2 = 14,
};

}  // namespace mbzirc
#endif  // MBZIRCTYPES_HH_
