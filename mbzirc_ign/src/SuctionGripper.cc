#include <ignition/plugin/Register.hh>


#include <ignition/transport/Node.hh>

#include <ignition/msgs/contacts.pb.h>

#include <ignition/msgs.hh>

#include <ignition/gazebo/components.hh>
#include <ignition/gazebo/Model.hh>

#include "SuctionGripper.hh"

using namespace mbzirc;
using namespace ignition;
using namespace gazebo;

class mbzirc::SuctionGripperPrivate
{
  /// \brief The item being moved
  public: Entity childItem{kNullEntity};

  /// \brief The gripper link name
  public: std::string linkName;

  /// \brief Used to store the joint when we attach to an object
  public: Entity joint{kNullEntity};

  /// \brief The gripper link entity
  public: Entity gripperEntity{kNullEntity};

  /// \brief The transport node
  public: transport::Node node;

  /// \brief Used for determining when the suction is on.
  public: bool suctionOn{true};

  /// \brief Set to true when we detect the suction gripper is in contact
  public: bool pendingJointCreation{false};

  /// \brief True when we are holding an object
  public: bool jointCreated{false};

  /// \brief mutex for accessing member variables
  public: std::mutex mtx;

  /// \brief Callback for when contact is made
  public: void OnContact(const ignition::msgs::Contacts &_msg)
  {
    // If the suction is not on or the gripper is already holding an object
    // dont hold an object.
    if (!this->suctionOn || this->jointCreated) return;

    // Grasp an aobject
    // TODO(arjo): What happens when there are many objects in contact
    for (int i = 0; i < _msg.contact_size(); ++i)
    {
      auto contact = _msg.contact(i);
      std::lock_guard<std::mutex> lock(this->mtx);
      this->childItem = contact.collision2().id();
      pendingJointCreation = true;
    }
  }

  /// \brief Command callback
  public: void OnCmd(const ignition::msgs::Boolean &_suctionOn)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->suctionOn = _suctionOn.data();
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
  if(_sdf->HasElement("parent_link"))
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
    ignerr << "Could not find link named "
      << this->dataPtr->linkName << std::endl;
    return;
  }

  if(_sdf->HasElement("contact_sensor_topic"))
  {
    auto topic = _sdf->Get<std::string>("contact_sensor_topic");
    this->dataPtr->node.Subscribe(
      topic,
      &SuctionGripperPrivate::OnContact,
      this->dataPtr.get());
  }
  else
  {
    ignerr << "Please specify a contact_sensor_topic" << std::endl;
    return;
  }

  if(_sdf->HasElement("command_topic"))
  {
    auto topic = _sdf->Get<std::string>("command_topic");
    this->dataPtr->node.Subscribe(
      topic,
      &SuctionGripperPrivate::OnCmd,
      this->dataPtr.get());
  }
  else
  {
    ignerr << "Please specify a command_topic" << std::endl;
    return;
  }
}


//////////////////////////////////////////////////
void SuctionGripperPlugin::PreUpdate(const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (_info.paused) return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
  if (this->dataPtr->pendingJointCreation)
  {
    // If we need to create a new joint
    this->dataPtr->pendingJointCreation = false;
    this->dataPtr->joint = _ecm.CreateEntity();
    auto parentLink = _ecm.ParentEntity(this->dataPtr->childItem);
    _ecm.CreateComponent(
          this->dataPtr->joint,
          components::DetachableJoint({this->dataPtr->gripperEntity,
                                       parentLink, "fixed"}));
    igndbg << "Created joint between gripper and "
      << this->dataPtr->childItem
      << std::endl << "at time step " << _info.simTime.count() << std::endl;
    this->dataPtr->jointCreated = true;
  }

  if (!this->dataPtr->suctionOn && this->dataPtr->jointCreated)
  {
    // If e have an item and were commanded to release it
    _ecm.RequestRemoveEntity(this->dataPtr->joint);
    this->dataPtr->joint = kNullEntity;
    this->dataPtr->jointCreated = false;
    igndbg << "Remove joint between gripper and "
      << this->dataPtr->childItem
      << std::endl << "at time step " << _info.simTime.count() << std::endl;

  }
}


IGNITION_ADD_PLUGIN(
  mbzirc::SuctionGripperPlugin,
  ignition::gazebo::System,
  mbzirc::SuctionGripperPlugin::ISystemConfigure,
  mbzirc::SuctionGripperPlugin::ISystemPreUpdate)