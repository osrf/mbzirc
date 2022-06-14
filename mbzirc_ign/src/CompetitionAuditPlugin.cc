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

#include <ignition/plugin/Register.hh>

#include "CompetitionAuditPlugin.hh"
#include "Components.hh"

IGNITION_ADD_PLUGIN(
    mbzirc::CompetitionAuditPlugin,
    ignition::gazebo::System)

using Entity = ignition::gazebo::Entity;


namespace mbzirc {

class CompetitionAuditPluginPrivate
{
  /// \brief Names of the spawned robots.
  public: std::unordered_map<Entity, std::string> robotNames;

  /// \brief Initial pose of robots
  public: std::unordered_map<Entity, ignition::math::Vector3d> robotInitialPos;
};

//////////////////////////////////////////////////
CompetitionAuditPlugin::CompetitionAuditPlugin()
  : dataPtr(new CompetitionAuditPluginPrivate)
{
}

//////////////////////////////////////////////////
CompetitionAuditPlugin::~CompetitionAuditPlugin()
{
}

//////////////////////////////////////////////////
void CompetitionAuditPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{

}

//////////////////////////////////////////////////
void CompetitionAuditPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{

}

}  // namespace mbzirc
