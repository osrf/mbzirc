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

#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/sensors/Noise.hh>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include "NaiveRadar.hh"

using namespace mbzirc;

/////////////////////////////////////////////////
NaiveRadar::NaiveRadar()
{
}

/////////////////////////////////////////////////
NaiveRadar::~NaiveRadar() = default;

//////////////////////////////////////////////////
void NaiveRadar::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
  // parse configuration parameters from SDF
  auto sdf = const_cast<sdf::Element *>(_sdf.get());
  this->updateRate = sdf->Get("update_rate", this->updateRate).first;
  if (sdf->HasElement("scan"))
  {
    sdf::ElementPtr scanElem = sdf->GetElement("scan");
    sdf::ElementPtr horElem = scanElem->GetElement("horizontal");
    this->minAngle = horElem->Get("min_angle", this->minAngle).first;
    this->maxAngle = horElem->Get("max_angle", this->maxAngle).first;
    sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
    this->minVerticalAngle =
        vertElem->Get("min_angle", this->minVerticalAngle).first;
    this->maxVerticalAngle =
        vertElem->Get("max_angle", this->maxVerticalAngle).first;
    sdf::ElementPtr rangeElem = scanElem->GetElement("range");
    this->minRange = rangeElem->Get("min", this->minRange).first;
    this->maxRange = rangeElem->Get("max", this->maxRange).first;
    if (scanElem->HasElement("noise"))
    {
      sdf::ElementPtr noiseElem = scanElem->GetElement("noise");
      this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseElem);
    }
  }

  // Get top level model this entity belongs to
  // this system is attached to a sensor model but we are only interested
  // in the top level vehicle model pose - this is later used for
  // computing distance to other models in the environment
  auto parent = _ecm.Component<ignition::gazebo::components::ParentEntity>(
      _entity);
  this->entity = _entity;
  this->modelEntity = _entity;
  while (parent && _ecm.Component<ignition::gazebo::components::Model>(
         parent->Data()))
  {
    this->modelEntity = parent->Data();
    parent = _ecm.Component<ignition::gazebo::components::ParentEntity>(
        parent->Data());
  }

  // set topic to publish sensor data to
  std::string topic = ignition::gazebo::scopedName(_entity, _ecm) + "/radar/scan";
  topic = ignition::transport::TopicUtils::AsValidTopic(topic);

  // create the publisher
  this->publisher =
    this->node.Advertise<ignition::msgs::Float_V>(topic);
}

//////////////////////////////////////////////////
void NaiveRadar::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{

  // Throttle the sensor updates using sim time
  // If update_rate is set to 0, it means unthrottled
  if (_info.simTime < this->nextUpdateTime && this->updateRate > 0)
    return;

  if (this->updateRate > 0.0)
  {
    // Update the time the plugin should be loaded
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::duration<double>(1.0 / this->updateRate));
    this->nextUpdateTime += delta;
  }

  // do not bother generating data if there are no subscibers
  if (!this->publisher.HasConnections())
    return;

  // get the pose of the model
  const ignition::gazebo::components::Pose *poseComp =
      _ecm.Component<ignition::gazebo::components::Pose>(this->modelEntity);
  ignition::math::Pose3d entityPose = poseComp->Data();

  // compute the inverse of the pose
  // this is used later to convert pose of other entites into this model frame
  auto inversePose = entityPose.Inverse();

  // create vectors to store data that will be published by the sensor
  std::vector<double> ranges;
  std::vector<double> azimuths;
  std::vector<double> elevations;

  // Loop through all the models in simulation
  _ecm.Each<ignition::gazebo::components::Model,
            ignition::gazebo::components::Pose,
            ignition::gazebo::components::ParentEntity>(
      [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Model *,
          const ignition::gazebo::components::Pose *_pose,
          const ignition::gazebo::components::ParentEntity *_parent) -> bool
      {
        // skip self
        if (_entity == this->modelEntity)
          return true;

        // ignore nested models by checking to see if it has a parent entity
        // that is also a model
        const ignition::gazebo::components::ParentEntity *parentComp =
            _ecm.Component<ignition::gazebo::components::ParentEntity>(
            _parent->Data());
        if (parentComp &&
            _ecm.Component<ignition::gazebo::components::Model>(
            parentComp->Data()))
          return true;

        // get the model pose
        ignition::math::Pose3d pose = _pose->Data();

        // compute range
        double range = entityPose.Pos().Distance(pose.Pos());

        // discard data that are out of range
        if (range > this->maxRange || range < this->minRange)
          return true;

        // rotate entity pose to model frame
        // The position is now also the direction
        ignition::math::Vector3d dir = (inversePose * pose).Pos();
        dir.Normalize();

        // compute azimuth and elevation angles
        ignition::math::Vector3d xy(dir.X(), dir.Y(), 0.0);
        xy.Normalize();
        double azimuth = std::acos(ignition::math::Vector3d::UnitX.Dot(xy));
        azimuth = (dir.Y() < 0) ? -azimuth : azimuth;

        // filter out models that are outside the min/max angles
        if (azimuth > this->maxAngle || azimuth < this->minAngle)
          return true;

        double elevation = std::acos(xy.Dot(dir));
        elevation = (dir.Z() < 0) ? -elevation : elevation;

        // filter out models that are outside the min/max vertical angles
        if (elevation > maxVerticalAngle || elevation < minVerticalAngle)
          return true;

        // apply noise
        if (this->noise)
        {
          range = this->noise->Apply(range);
          azimuth = this->noise->Apply(azimuth);
          elevation = this->noise->Apply(elevation);
        }

        // store results to be published
        ranges.push_back(range);
        azimuths.push_back(azimuth);
        elevations.push_back(elevation);

        return true;
      });

  // populate and publish the message

  // time stamp the message with sim time
  ignition::msgs::Float_V msg;
  *msg.mutable_header()->mutable_stamp() =
      ignition::msgs::Convert(_info.simTime);
  auto frame = msg.mutable_header()->add_data();

  // set frame id to scoped name of this sensor
  frame->set_key("frame_id");
  std::string scopedName =
      ignition::gazebo::removeParentScope(
      ignition::gazebo::scopedName(this->entity, _ecm, "::", false), "::");
  frame->add_value(scopedName);

  // populate sensor data
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    msg.add_data(ranges[i]);
    msg.add_data(azimuths[i]);
    msg.add_data(elevations[i]);
  }
  this->publisher.Publish(msg);
}

// Register the plugin
IGNITION_ADD_PLUGIN(mbzirc::NaiveRadar,
                    ignition::gazebo::System,
                    NaiveRadar::ISystemConfigure,
                    NaiveRadar::ISystemPostUpdate)
