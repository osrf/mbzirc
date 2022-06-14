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

#include <ignition/msgs/param_v.pb.h>

#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Util.hh"

#include "RFRange.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Parameters for simple log-normal fading model.
struct RangeConfiguration
{
  /// \brief Hard limit on range.
  double maxRange = 50.0;

  /// \brief Fading exponent.
  double fadingExponent = 2.5;

  /// \brief Received power at 1m (in dBm).
  double l0 = 40;

  /// \brief Standard deviation for received power.
  double sigma = 10;

  /// \brief measured rssi at 1 meter.
  double rssi1 = -15;

  /// Output stream operator.
  /// \param[out] _oss Stream.
  /// \param[in] _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RangeConfiguration &_config)
  {
    _oss << "RF Configuration (range-based)" << std::endl
         << "-- max_range: " << _config.maxRange << std::endl
         << "-- fading_exponent: " << _config.fadingExponent << std::endl
         << "-- l0: " << _config.l0 << std::endl
         << "-- sigma: " << _config.sigma << std::endl
         << "-- rssi_1: " << _config.rssi1 << std::endl;

    return _oss;
  }
};

/// \brief Radio configuration parameters.
///
/// Static parameters such as channel capacity and transmit power.
struct RadioConfiguration
{
  /// \brief Capacity of radio in bits-per-second.
  double capacity = 54000000;

  /// \brief Default transmit power in dBm. Default is 27dBm or 500mW.
  double txPower = 27;

  /// \brief Modulation scheme, e.g., QPSK (Quadrature Phase Shift Keyring).
  std::string modulation = "QPSK";

  /// \brief Noise floor of the radio in dBm.
  double noiseFloor = -90;

  /// Output stream operator.
  /// \param _oss Stream.
  /// \param _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RadioConfiguration &_config)
  {
    _oss << "Radio Configuration" << std::endl
         << "-- capacity: " << _config.capacity << std::endl
         << "-- tx_power: " << _config.txPower << std::endl
         << "-- noise_floor: " << _config.noiseFloor << std::endl
         << "-- modulation: " << _config.modulation << std::endl;

    return _oss;
  }
};

/// \brief Type for holding RF power as a Normally distributed random variable.
struct RFPower
{
  /// \brief Expected value of RF power.
  double mean;

  /// \brief Variance of RF power.
  double variance;

  /// \brief double operator.
  /// \return the RFPower as a double.
  operator double() const
  {
    return mean;
  }
};

//////////////////////////////////////////////////
RFRangeSensor::RFRangeSensor()
{
  ignerr << "RFRangeSensor constructor" << std::endl;
}

//////////////////////////////////////////////////
RFRangeSensor::~RFRangeSensor() = default;

//////////////////////////////////////////////////
void RFRangeSensor::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto entity = _ecm.CreateEntity();

  _ecm.SetParentEntity(entity, _entity);
  _ecm.CreateComponent(entity, RFRangeType());

  math::Pose3d pose;
  if (_ecm.EntityHasComponentType(_entity, gazebo::components::Model::typeId))
  {
    auto modelPose = _ecm.Component<gazebo::components::Pose>(_entity);
    pose += modelPose->Data();
    _ecm.CreateComponent(entity, gazebo::components::WorldPose(pose));
  }
}

/// \brief ToDo.
class gazebo::systems::RFRangePrivate
{
  /// \brief ToDo.
  public: struct RFRangeData
  {
    /// \brief Sensor name.
    public: std::string name;

    /// \brief Sensor pose.
    public: math::Pose3d pose;

    /// \brief An Ignition Transport publisher.
    public: transport::Node::Publisher pub;
  };

  /// \brief ToDo.
  public: void Load(std::shared_ptr<const sdf::Element> _sdf);

  /// \brief ToDo.
  public: double DbmToPow(double _dBm) const;

  /// \brief ToDo.
  public: double QPSKPowerToBER(double _power, double _noise) const;

  /// \brief ToDo.
  public: RFPower LogNormalReceivedPower(const double &_txPower,
    const math::Vector3d &_txPos, const math::Vector3d &_rxPos) const;

  /// \brief ToDo.
  public: double RSSIToRange(double rssi) const;

  /// \brief Attempt communication between two nodes.
  ///
  /// The radio configuration, transmitter and receiver state, and
  /// packet size are all used to compute the probability of successful
  /// communication (i.e., based on both SNR and bitrate
  /// limitations). This probability is then used to determine if the
  /// packet is successfully communicated.
  ///
  /// \param[in out] _txState Current state of the transmitter.
  /// \param[in out] _rxState Current state of the receiver.
  /// \param[in] _numBytes Size of the packet.
  /// \return std::tuple<bool, double> reporting if the packet should be
  /// delivered and the received signal strength (in dBm).
  public: std::tuple<bool, double> AttemptSend(math::Vector3d &_txPos,
                                               math::Vector3d &_rxPos,
                                               const uint64_t &_numBytes);

  /// \brief ToDo.
  public: void UpdateSensorPoses(gazebo::EntityComponentManager &_ecm);

  /// \brief Todo.
  public: std::string RemoveAllScope(const std::string &_name,
                                     const std::string &_delim);

  /// \brief An Ignition Transport node for communications.
  public: transport::Node node;

  /// \brief Last system update simulation time.
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief System update period calculated from <update_rate>.
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief ToDO.
  public: std::map<gazebo::Entity, RFRangeData> entityMap;

  /// \brief Range configuration.
  public: RangeConfiguration rangeConfig;

  /// \brief Radio configuration.
  public: RadioConfiguration radioConfig;

  /// \brief Random device to seed random engine
  public: std::random_device rd{};

  /// \brief Random number generator.
  public: std::default_random_engine rndEngine{rd()};
};

//////////////////////////////////////////////////
void RFRangePrivate::Load(std::shared_ptr<const sdf::Element> _sdf)
{
  if (_sdf->HasElement("range_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("range_config");

    this->rangeConfig.maxRange =
      elem->Get<double>("max_range", this->rangeConfig.maxRange).first;

    this->rangeConfig.fadingExponent =
      elem->Get<double>("fading_exponent",
        this->rangeConfig.fadingExponent).first;

    this->rangeConfig.l0 =
      elem->Get<double>("l0", this->rangeConfig.l0).first;

    this->rangeConfig.sigma =
      elem->Get<double>("sigma", this->rangeConfig.sigma).first;

    this->rangeConfig.rssi1 =
      elem->Get<double>("rssi_1", this->rangeConfig.rssi1).first;
  }

  if (_sdf->HasElement("radio_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("radio_config");

    this->radioConfig.capacity =
      elem->Get<double>("capacity", this->radioConfig.capacity).first;

    this->radioConfig.txPower =
      elem->Get<double>("tx_power", this->radioConfig.txPower).first;

    this->radioConfig.modulation =
      elem->Get<std::string>("modulation",
        this->radioConfig.modulation).first;

    this->radioConfig.noiseFloor =
      elem->Get<double>("noise_floor",
        this->radioConfig.noiseFloor).first;
  }

  ignerr << "RFRange sensor range configuration:" << std::endl
         << this->rangeConfig << std::endl;

  ignerr << "RFRange sensor radio configuration:" << std::endl
         << this->radioConfig << std::endl;
}

/////////////////////////////////////////////
double RFRangePrivate::DbmToPow(double _dBm) const
{
  return 0.001 * pow(10., _dBm / 10.);
}

////////////////////////////////////////////
double RFRangePrivate::QPSKPowerToBER(
  double _power, double _noise) const
{
  return erfc(sqrt(_power / _noise));
}

/////////////////////////////////////////////
RFPower RFRangePrivate::LogNormalReceivedPower(
  const double &_txPower, const math::Vector3d &_txPos,
  const math::Vector3d &_rxPos) const
{
  const double kRange = _txPos.Distance(_rxPos);

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange)
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = this->rangeConfig.l0 +
    10 * this->rangeConfig.fadingExponent * log10(kRange);

  return {_txPower - kPL, this->rangeConfig.sigma};
}

/////////////////////////////////////////////
double RFRangePrivate::RSSIToRange(double rssi) const
{
  return std::pow(10,
    (this->rangeConfig.rssi1 - rssi) / (10 * this->rangeConfig.fadingExponent));
}

/////////////////////////////////////////////
std::tuple<bool, double> RFRangePrivate::AttemptSend(
  math::Vector3d &_txPos, math::Vector3d &_rxPos, const uint64_t &_numBytes)
{
  // Get the received power based on TX power and position of each node.
  auto rxPowerDist =
    this->LogNormalReceivedPower(this->radioConfig.txPower, _txPos, _rxPos);

  double rxPower = rxPowerDist.mean;
  if (rxPowerDist.variance > 0.0)
  {
    std::normal_distribution<> d{rxPowerDist.mean, sqrt(rxPowerDist.variance)};
    rxPower = d(this->rndEngine);
  }

  // Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER).
  double ber = this->QPSKPowerToBER(
    this->DbmToPow(rxPower), this->DbmToPow(this->radioConfig.noiseFloor));

  double packetDropProb = 1.0 - exp(_numBytes * log(1 - ber));

  // igndbg << "TX power (dBm): " << this->radioConfig.txPower << "\n" <<
  //           "RX power (dBm): " << rxPower << "\n" <<
  //           "BER: " << ber << "\n" <<
  //           "# Bytes: " << _numBytes << "\n" <<
  //           "PER: " << packetDropProb << std::endl;

  double randDraw = ignition::math::Rand::DblUniform();
  bool packetReceived = randDraw > packetDropProb;

  if (!packetReceived)
    return std::make_tuple(false, std::numeric_limits<double>::lowest());

  return std::make_tuple(true, rxPower);
}

//////////////////////////////////////////////////
void RFRangePrivate::UpdateSensorPoses(gazebo::EntityComponentManager &_ecm)
{
  for (auto &[entityId, data] : this->entityMap)
  {
    // Pose of the model.
    auto modelPose = _ecm.Component<components::Pose>(entityId);
    if (modelPose == nullptr)
      _ecm.CreateComponent(entityId, components::Pose());

    // We just created the joint position component, give one iteration for the
    // physics system to update its size
    if (modelPose == nullptr)
      continue;

    data.pose = modelPose->Data();
  }
}

//////////////////////////////////////////////////
std::string RFRangePrivate::RemoveAllScope(const std::string &_name,
  const std::string &_delim)
{
  auto sepPos = _name.rfind(_delim);
  if (sepPos == std::string::npos || (sepPos + _delim.size()) > _name.size())
    return _name;

  return _name.substr(sepPos + _delim.size());
}

//////////////////////////////////////////////////
RFRange::RFRange()
  : dataPtr(std::make_unique<RFRangePrivate>())
{
  ignerr << "RFRange initialized" << std::endl;
}

//////////////////////////////////////////////////
void RFRange::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->Load(_sdf);

  // Initialize system update period
  double rate = _sdf->Get<double>("update_rate", 1).first;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->dataPtr->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
}

//////////////////////////////////////////////////
void RFRange::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("RFRange::PreUpdate");

  _ecm.EachNew<RFRangeType,
               ignition::gazebo::components::WorldPose>(
    [&](const ignition::gazebo::Entity &_entity,
        const RFRangeType *_custom,
        const ignition::gazebo::components::WorldPose *_pose)->bool
      {
        // This is the ID of the model containing the sensor.
        gazebo::Entity sensorId = _ecm.ParentEntity(_entity);
        std::string sensorName = gazebo::scopedName(sensorId, _ecm);

        // This is the ID of the robot with the sensor attached.
        gazebo::Entity modelId = _ecm.ParentEntity(sensorId);
        std::string modelName = gazebo::scopedName(modelId, _ecm);

        // Keep track of this sensor.
        RFRangePrivate::RFRangeData rfRangeData;
        rfRangeData.name = this->dataPtr->RemoveAllScope(modelName, "/");
        rfRangeData.pose = math::Pose3d::Zero;
        rfRangeData.pub = this->dataPtr->node.Advertise<msgs::Param_V>(
          sensorName + "/rfsensor");

        // We store the ID of the robot model.
        this->dataPtr->entityMap[modelId] = rfRangeData;

        return true;
      });

  if (_info.paused)
    return;

  // Update the poses of all detected RF sensors.
  this->dataPtr->UpdateSensorPoses(_ecm);

  // Throttle update rate.
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->updatePeriod)
  {
    return;
  }
  this->dataPtr->lastUpdateTime = _info.simTime;

  for (auto &[from, dataFrom] : this->dataPtr->entityMap)
  {
    ignition::msgs::Param_V outputMsg;

    for (auto &[to, dataTo] : this->dataPtr->entityMap)
    {
      if (from == to)
        continue;

      auto [sendPacket, rssi] = this->dataPtr->AttemptSend(
        dataFrom.pose.Pos(), dataTo.pose.Pos(), 100);

      if (!sendPacket)
        continue;

      auto *param = outputMsg.add_param()->mutable_params();

      ignition::msgs::Any modelValue;
      modelValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_STRING);
      modelValue.set_string_value(dataTo.name);

      // Set the model field.
      (*param)["model"] = modelValue;

      double range = this->dataPtr->RSSIToRange(rssi);
      ignition::msgs::Any rangeValue;
      rangeValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_DOUBLE);
      rangeValue.set_double_value(range);

      // Set the playback field.
      (*param)["range"] = rangeValue;

      ignition::msgs::Any rssiValue;
      rssiValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_DOUBLE);
      rssiValue.set_double_value(rssi);

      // Set the playback field.
      (*param)["rssi"] = rssiValue;
    }

    // Publish output.
    if (outputMsg.param_size() > 0)
    {
      // Header.
      auto stamp = math::durationToSecNsec(_info.simTime);
      outputMsg.mutable_header()->mutable_stamp()->set_sec(stamp.first);
      outputMsg.mutable_header()->mutable_stamp()->set_nsec(stamp.second);

      dataFrom.pub.Publish(outputMsg);
    }
  }
}

IGNITION_ADD_PLUGIN(RFRange,
                    ignition::gazebo::System,
                    RFRange::ISystemConfigure,
                    RFRange::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFRange,
                          "ignition::gazebo::systems::RFRange")

IGNITION_ADD_PLUGIN(RFRangeSensor,
                    ignition::gazebo::System,
                    RFRangeSensor::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(RFRangeSensor,
                          "ignition::gazebo::systems::RFRangeSensor")
