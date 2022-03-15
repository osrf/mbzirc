#include <ignition/plugin/Register.hh>


#include <ignition/transport/Node.hh>

#include <ignition/msgs/contacts.pb.h>

#include <ignition/gazebo/components.hh>
using namespace mbzirc;
using namespace ignition;
using namespace gazebo;

class mbzirc::SuctionGripperPrivate
{
  public: Entity childItem;

  public: std::string linkName;

  public: Entity joint;

  public: Entity gripperEntity;

  public: transport::Node node;

  public: bool gripperOpen{true};

  public: bool pendingJointCreation{false};

  public: std::mutex _mtx

  public: void OnContact(const ignition::msgs::Contacts &_msg)
  {
    if (!gripperOpen) continue;

    for (int i = 0; i < _msg.contact_size(); ++i)
    {
      auto contact = _msg.contact(i);
      if (contact.collision1() == this->gripperEntity)
      {
        std::lock_guard<std::mutex> lock(this->_mtx);
        this->childItem = contact.collision2();
        pendingJointCreation = true;
      }
      if (contact.collision2() == this->gripperEntity)
      {
        std::lock_guard<std::mutex> lock(this->_mtx);
        this->childItem = contact.collision1();
        pendingJointCreation = true;
      }
    }
  }
};


//////////////////////////////////////////////////
SuctionGripperPlugin::SuctionGripperPlugin():
  dataPtr(new SuctionGripperPrivate)
{
}

//////////////////////////////////////////////////
SuctionGripperPlugin::~SuctionGripperPlugin()
{
}

//////////////////////////////////////////////////
void SuctionGripperPlugin::Configure(const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  if(_sdf->hasElement("parent_link"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("parent_link");
  }
  else
  {
    ignerr << "Please specify a link name" << std::endl;
    return;
  }

  Model model(_entity);
  this->dataPtr->gripperEntity = model.LinkByName(_ecm, this->dataPtr->linkName);
  if (this->dataPtr->gripperEntity == kNullEntity)
  {
    ignerr << "Could not find link named " << this->dataPtr->linkName << std::endl;
    return;
  }

  if(_sdf->hasElement("contact_sensor_topic"))
  {
    auto topic = _sdf->Get<std::string>("contact_sensor_topic");
    this->dataPtr->node.Subscribe(topic, &SuctionGripperPrivate::OnContact, this);
  }
  else
  {
    ignerr << "Please specify a topic" << std::endl;
    return;
  }
  if(_sdf->hasElement("contact_sensor_topic"))
  {
    auto topic = _sdf->Get<std::string>("contact_sensor_topic");
    this->dataPtr->node.Subscribe(topic, &SuctionGripperPrivate::OnContact, this);
  }
  else
  {
    ignerr << "Please specify a topic" << std::endl;
    return;
  }
}
}

//////////////////////////////////////////////////
void SuctionGripperPlugin::PreUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->_mtx);
  if (this->dataPtr->pendingJointCreation)
  {
    this->dataPtr->pendingJointCreation = false;
    this->dataPtr->joint = _ecm.CreateEntity();
    _ecm.CreateComponent(
          this->detachableJointEntity,
          components::DetachableJoint({this->parentLinkEntity,
                                        this->childLinkEntity, "fixed"}));
  }
}


IGNITION_ADD_PLUGIN(
  mbzirc::SuctionGripperPlugin,
  ignition::gazebo::System,
  mbzirc::SuctionGripperPlugin::ISystemConfigure,
  mbzirc::SuctionGripperPlugin::ISystemPreUpdate)