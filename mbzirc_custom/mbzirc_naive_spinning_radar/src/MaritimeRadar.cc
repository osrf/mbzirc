#include "MaritimeRadar.hh"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/plugin/Register.hh>

using namespace mbzirc;
using namespace ignition;
using namespace gazebo;

///////////////////////////////////////////////////
MaritimeRadar::MaritimeRadar()
{

}

///////////////////////////////////////////////////
MaritimeRadar::~MaritimeRadar()
{

}

///////////////////////////////////////////////////
void MaritimeRadar::Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr)
{
  gazebo::Model model(_entity);

  // Get world name
  const auto worldEntity = _ecm.EntityByComponents(components::World());
  const std::string worldName =
    _ecm.Component<components::Name>(worldEntity)->Data();

  // Get the entity name
  const std::string entityName = ignition::gazebo::scopedName(_entity, _ecm);
  this->frameId =
    ignition::gazebo::removeParentScope(
      ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");

  // Get the laser topic
  this->laserTopic =
    "/world/" + worldName + "/" + entityName +
    "/link/sensor_link/sensor/gpu_lidar/scan";
  if (_sdf->HasElement("laser_topic"))
  {
    this->laserTopic = _sdf->Get<std::string>("laser_topic");
  }

  // Get the joint name
  auto jointName = _sdf->Get<std::string>("joint_name");
  this->jointEntity = model.JointByName(_ecm, jointName);

  // Create radar scan publisher
  if (_sdf->HasElement("radar_scan_topic"))
  {
    this->radarScanTopic= _sdf->Get<std::string>("radar_scan_topic");
  }
  else
  {
    // set topic to publish sensor data to
    std::string topic = entityName + "/radar/scan";
    this->radarScanTopic = ignition::transport::TopicUtils::AsValidTopic(topic);
  }
  this->radarScanPub =
    this->node.Advertise<msgs::Float_V>(this->radarScanTopic);
}

///////////////////////////////////////////////////
void MaritimeRadar::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto jointPosComp =
    _ecm.Component<components::JointPosition>(this->jointEntity);
  if (jointPosComp == nullptr)
  {
    ignerr << "No joint position component found for entity ["
           << this->jointEntity << "]\n";
    return;
  }
  // We just created the joint position component, give one iteration for the
  // physics system to update its size
  if (jointPosComp == nullptr || jointPosComp->Data().empty())
    return;

  // Sanity check: Make sure that the joint has an index
  if (jointPosComp->Data().size() == 0)
  {
    return;
  }

  if (!this->laserSubscribed && this->radarScanPub.HasConnections())
  {
    this->node.Subscribe(this->laserTopic, &MaritimeRadar::OnRadarScan, this);
    this->laserSubscribed = true;
  }
  else if (this->laserSubscribed && !this->radarScanPub.HasConnections())
  {
    this->node.Unsubscribe(this->laserTopic);
    this->laserSubscribed = false;
  }

  std::lock_guard<std::mutex> lock(this->mtx);
  this->radarSpokeAngle = fmod(jointPosComp->Data()[0], 2 * IGN_PI);
}

///////////////////////////////////////////////////
void MaritimeRadar::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // We need to track the joint position in order to know when to publish
  auto jointPosComp =
    _ecm.Component<components::JointPosition>(this->jointEntity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(jointEntity, components::JointPosition());
  }
}

///////////////////////////////////////////////////
void MaritimeRadar::OnRadarScan(const ignition::msgs::LaserScan &_msg)
{
  double currRadarSpokeAngle;
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    currRadarSpokeAngle = radarSpokeAngle;
  }

  ignition::msgs::Float_V radarScanMsg;
  *radarScanMsg.mutable_header()->mutable_stamp() = _msg.header().stamp();
  auto frame = radarScanMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->frameId);

  auto wrapAngles = [](double input)
  {
    if (input < -IGN_PI)
    {
      return 2 * IGN_PI + input;
    }
    else if (input > IGN_PI)
    {
      return -2 * IGN_PI - input;
    }
    return input;
  };

  // Start with -1 x vertical angle step so we just need additions onwards
  double curr_elevation =
    wrapAngles(_msg.vertical_angle_min() - _msg.vertical_angle_step());
  for (uint32_t i = 0; i < _msg.vertical_count(); ++i)
  {
    int ranges_before_channel = i * _msg.count();
    curr_elevation += _msg.vertical_angle_step();

    // Start with -1 x angle step so we just need additions onwards
    double curr_azimuth =
      _msg.angle_min() - _msg.angle_step() + currRadarSpokeAngle;
    for (uint32_t j = 0; j < _msg.count(); ++j)
    {
      curr_azimuth = wrapAngles(curr_azimuth + _msg.angle_step());
      double curr_range = _msg.ranges(ranges_before_channel + j);

      if (curr_range < _msg.range_min() || curr_range > _msg.range_max())
        continue;

      radarScanMsg.add_data(curr_range);
      radarScanMsg.add_data(curr_azimuth);
      radarScanMsg.add_data(curr_elevation);
    }
  }
  radarScanPub.Publish(radarScanMsg);
}

IGNITION_ADD_PLUGIN(mbzirc::MaritimeRadar,
                    ignition::gazebo::System,
                    MaritimeRadar::ISystemConfigure,
                    MaritimeRadar::ISystemPreUpdate,
                    MaritimeRadar::ISystemPostUpdate)
