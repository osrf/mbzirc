#include "MaritimeRadar.hh"

#include <ignition/gazebo/Model.hh>
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

  // TODO(arjo): This is a hack to get the laser
  std::string laserTopic =
    "/world/coast/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan";
  if (_sdf->HasElement("laser_topic"))
  {
    laserTopic = _sdf->Get<std::string>("laser_topic");
  }
  node.Subscribe(laserTopic,
    &MaritimeRadar::OnRadarScan, this);

  // Get the joint name
  auto jointName = _sdf->Get<std::string>("joint_name");
  this->jointEntity = model.JointByName(_ecm, jointName);

  // Get the angular resolution
  if(_sdf->HasElement("angular_resolution"))
  {
    this->angularResolution = _sdf->Get<double>("angular_resolution");
  }

  // Get the linear resolution
  if(_sdf->HasElement("linear_resolution"))
  {
    this->resolution = _sdf->Get<double>("linear_resolution");
  }

  // Get the maximum range
  if(_sdf->HasElement("max_range"))
  {
    this->angularResolution = _sdf->Get<double>("max_range");
  }

  // Get the minimum range
  if(_sdf->HasElement("min_range"))
  {
    this->angularResolution = _sdf->Get<double>("min_range");
  }

  // Get range buckets
  std::size_t numRangeBuckets = floor(maxRange/resolution) + 1;
  std::size_t numAngularBuckets = ceil(2 * IGN_PI /angularResolution) + 1;
  radarBin.resize(numAngularBuckets);

  for (std::size_t i = 0; i < numAngularBuckets; ++i)
  {
    radarBin[i].resize(numRangeBuckets, 0);
  }

  // Create debug image
  // TODO(arjo): Make this configuarable.
  this->image.set_width(this->debugImageWidth);
  this->image.set_height(this->debugImageWidth);
  this->image.set_step(this->debugImageWidth);
  this->image.set_pixel_format_type(msgs::PixelFormatType::L_INT8);
  this->image.mutable_data()->resize(
    this->debugImageWidth * this->debugImageWidth);

  // Create debug publisher
  if (_sdf->HasElement("debug_image_topic"))
  {
    this->debugImageTopic = _sdf->Get<std::string>("debug_image_topic");
  }
  this->debugPub  = this->node.Advertise<msgs::Image>(debugImageTopic);

  // Create radar spoke publisher
  if (_sdf->HasElement("radar_spoke_topic"))
  {
    this->radarSpokeTopic = _sdf->Get<std::string>("radar_spoke_topic");
  }
  this->linePub  = this->node.Advertise<msgs::Float_V>(this->radarSpokeTopic);
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

  std::size_t index = floor(
    fmod(jointPosComp->Data()[0], 2 * IGN_PI) /  angularResolution);

  std::lock_guard<std::mutex> lock(this->mtx);
  if (index != this->radarBinIndex)
  {
    this->PublishScan();
    this->ClearScanBuffer((index) % this->radarBin.size());
    this->radarBinIndex = index;
    this->numBeams = 0;
  }
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
void MaritimeRadar::PublishScan()
{
  if(this->radarBin.size() < radarBinIndex +1)
    return;
  // Publish just a single radar bin
  //
  ignition::msgs::Float_V lineBin;
  // First field of the float array is the angle
  lineBin.add_data(this->radarBinIndex * this->angularResolution);
  // Second field is the angular resolution
  lineBin.add_data(this->angularResolution);
  // Third field is the linear resolution
  lineBin.add_data(this->resolution);
  // Rest of the fields are the range bins
  for (std::size_t i = 0; i < this->radarBin[radarBinIndex].size(); ++i)
  {
    // Convert to DB reflection
    // Equation can be found here: https://en.wikipedia.org/wiki/Decibel
    // TODO(arjo): Add some noise to this.
    // Sea state vs radar noise model can be found here:
    // https://www.mathworks.com/help/radar/ug/sea-clutter-simulation-for-maritime-radar-system.html
    lineBin.add_data(
      10 * log10(this->radarBin[radarBinIndex][i]/this->numBeams));
  }
  linePub.Publish(lineBin);

  // The code below this point is for publishing debug images with a polar plot
  // of radar hits/misses
  // Convert to a message and publish
  if(this->radarBinIndex == 0)
  {
    // Reset whenever we go back to 0
    for (int i = 0; i < this->image.data().size(); i++)
    {
      (*this->image.mutable_data())[i] = 0;
    }
  }

  // Plot current scan line
  // TODO(arjo): plot using bressenham algorithm for cleaner lines
  for (int j = 0; j  < this->radarBin[radarBinIndex].size(); j++)
  {
    double x =
      cos(radarBinIndex * this->angularResolution)
      * debugImageWidth/2 * (j * resolution / maxRange);
    double y =
      sin(radarBinIndex * this->angularResolution)
      * debugImageWidth/2 * (j * resolution / maxRange);

    std::size_t pixel_x = x + debugImageWidth/2;
    std::size_t pixel_y = y + debugImageWidth/2;

    std::size_t index = pixel_x * this->image.step() + pixel_y;
    if (this->image.data().size() > index)
    {
      if(this->radarBin[radarBinIndex][j] > 0 || (*this->image.mutable_data())[index] > 0)
        (*this->image.mutable_data())[index] = 255;
    }
  }
  this->debugPub.Publish(this->image);
}

///////////////////////////////////////////////////
void MaritimeRadar::ClearScanBuffer(std::size_t _index)
{
  // Clear the radar buffer for given index
  for (std::size_t j = 0; j < radarBin[_index].size(); ++j)
  {
    radarBin[_index][j] = 0;
  }
}

///////////////////////////////////////////////////
void MaritimeRadar::OnRadarScan(const ignition::msgs::LaserScan &_msg)
{
  // We only want one radarbin
  std::lock_guard<std::mutex> lock(this->mtx);

  // Take the point cloud and project it onto a polar 2D plot.
  auto minAngle = _msg.angle_min();
  auto angle = minAngle;
  this->numBeams += _msg.ranges_size();
  for (int  i = 0; i < _msg.ranges_size(); ++i)
  {
    double range = _msg.ranges(i);
    angle+=this->angularResolution;
    auto projectedRange = cos(angle)*range;
    if (projectedRange > maxRange || projectedRange < minRange)
      continue;

    // Register the range in the radar bin
    this->radarBin[this->radarBinIndex][floor(projectedRange/resolution)]++;
  }
}

IGNITION_ADD_PLUGIN(mbzirc::MaritimeRadar,
                    ignition::gazebo::System,
                    MaritimeRadar::ISystemConfigure,
                    MaritimeRadar::ISystemPreUpdate,
                    MaritimeRadar::ISystemPostUpdate)