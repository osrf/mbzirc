#ifndef MBZIRC_IGN_GAMELOGICPLUGIN_HH_
#define MBZIRC_IGN_GAMELOGICPLUGIN_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace mbzirc
{
  class FixedWingControllerPrivate;

  /// \brief A plugin that takes care of fixed wing control of the zephyr.
  class FixedWingControllerPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: FixedWingControllerPlugin();

    /// \brief Destructor
    public: ~FixedWingControllerPlugin() override;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<FixedWingControllerPrivate> dataPtr;
  };
}
#endif