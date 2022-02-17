#include "FixedWingController.hh"

#include <ignition/math.hh>

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/plugin/Register.hh>

using namespace mbzirc;
using namespace ignition;
using namespace gazebo;

class mbzirc::FixedWingControllerPrivate
{
  public: ignition::math::PID rollControl;

  public: ignition::math::PID pitchControl;

  public: ignition::math::PID velocityControl;

  public: ignition::gazebo::Entity entity;
};

FixedWingControllerPlugin::FixedWingControllerPlugin() :
  dataPtr(std::make_unique<FixedWingControllerPrivate>())
{
}

FixedWingControllerPlugin::~FixedWingControllerPlugin()
{
}

void FixedWingControllerPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;

  enableComponent<components::Pose>(_ecm, _entity);
  enableComponent<components::WorldLinearVelocity>(_ecm, _entity);
  enableComponent<components::WorldAngularVelocity>(_ecm, _entity);
  enableComponent<components::LinearVelocity>(_ecm, _entity);
  enableComponent<components::AngularVelocity>(_ecm, _entity);
}

void FixedWingControllerPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto pose = _ecm.Component<components::Pose>(this->dataPtr->entity);
  auto rotation = pose->Data().Rot().Euler();
  igndbg << rotation <<std::endl;
}

IGNITION_ADD_PLUGIN(
    mbzirc::FixedWingControllerPlugin,
    ignition::gazebo::System,
    mbzirc::FixedWingControllerPlugin::ISystemConfigure,
    mbzirc::FixedWingControllerPlugin::ISystemPreUpdate)
