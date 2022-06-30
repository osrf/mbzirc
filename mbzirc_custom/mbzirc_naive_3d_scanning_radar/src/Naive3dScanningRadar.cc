#include "Naive3dScanningRadar.hh"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>

using namespace mbzirc;
using namespace ignition;
using namespace gazebo;

///////////////////////////////////////////////////
Naive3dScanningRadar::Naive3dScanningRadar()
{

}

///////////////////////////////////////////////////
Naive3dScanningRadar::~Naive3dScanningRadar()
{
}

///////////////////////////////////////////////////
void Naive3dScanningRadar::Configure(const ignition::gazebo::Entity &_entity,
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
    "/link/base_link/sensor/gpu_lidar/scan";
  if (_sdf->HasElement("laser_topic"))
  {
    this->laserTopic = _sdf->Get<std::string>("laser_topic");
  }

  // Create radar scan publisher
  if (_sdf->HasElement("radar_scan_topic"))
  {
    this->radarScanTopic = _sdf->Get<std::string>("radar_scan_topic");
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
void Naive3dScanningRadar::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->laserSubscribed && this->radarScanPub.HasConnections())
  {
    this->node.Subscribe(
      this->laserTopic, &Naive3dScanningRadar::OnRadarScan, this);
    this->laserSubscribed = true;
  }
  else if (this->laserSubscribed && !this->radarScanPub.HasConnections())
  {
    this->node.Unsubscribe(this->laserTopic);
    this->laserSubscribed = false;
  }
}

///////////////////////////////////////////////////
void Naive3dScanningRadar::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;
}

///////////////////////////////////////////////////
void Naive3dScanningRadar::OnRadarScan(const ignition::msgs::LaserScan &_msg)
{
  ignition::msgs::Float_V radarScanMsg;
  *radarScanMsg.mutable_header()->mutable_stamp() = _msg.header().stamp();
  auto frame = radarScanMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->frameId);

  // \todo(anyone) make this configurable
  int subsampleSize = 6;

  // We use a lidar with  higher number of samples. We then downsample
  // by computing an average of range values within a cluster of points
  // (subsampleSize). This is done to simulate a radar "beam" that has a
  // a beam width so that we are not just sampling using a ray.

  // Start with -1 x vertical angle step so we just need additions onwards
  double curr_elevation =
    _msg.vertical_angle_min() - _msg.vertical_angle_step();
  for (uint32_t i = 0; i < _msg.vertical_count(); ++i)
  {
    int ranges_before_channel = i * _msg.count();
    curr_elevation += _msg.vertical_angle_step();

    // Start with -1 x angle step so we just need additions onwards
    double curr_azimuth = _msg.angle_min() - _msg.angle_step();
    for (uint32_t j = 0; j < _msg.count(); j += subsampleSize)
    {
      double azimuth = 0.0;
      double range = ignition::math::INF_D;
      unsigned int rangeSampleCount = 0u;

      // loop through cluster of points and get avg azimuth and range
      for (unsigned int k = 0; k < subsampleSize; ++k)
      {
        curr_azimuth += _msg.angle_step();
        azimuth += curr_azimuth;
        double r = _msg.ranges(ranges_before_channel + j + k);
        // filter out inf range values so we compute avg range only from
        // valid range values
        if (r < _msg.range_min() || r > _msg.range_max())
          continue;
        if (rangeSampleCount == 0u)
        {
          range = r;
        }
        else
        {
          range += r;
          rangeSampleCount++;
        }
      }

      // compute avg range
      if (rangeSampleCount > 0)
        range = range / rangeSampleCount;

      // don't publish inf range data
      if (range < _msg.range_min() || range > _msg.range_max())
        continue;

      // compute current avg azimuth
      azimuth = azimuth / subsampleSize;

      radarScanMsg.add_data(range);
      radarScanMsg.add_data(azimuth);
      radarScanMsg.add_data(curr_elevation);
    }
  }
  radarScanPub.Publish(radarScanMsg);
}

IGNITION_ADD_PLUGIN(mbzirc::Naive3dScanningRadar,
                    ignition::gazebo::System,
                    Naive3dScanningRadar::ISystemConfigure,
                    Naive3dScanningRadar::ISystemPreUpdate,
                    Naive3dScanningRadar::ISystemPostUpdate)
