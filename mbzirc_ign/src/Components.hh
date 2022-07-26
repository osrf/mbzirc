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

#ifndef MBZIRC_IGN__COMPONENTS_HH_
#define MBZIRC_IGN__COMPONENTS_HH_

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>

namespace mbzirc
{
namespace components
{
  using NoData = ignition::gazebo::components::NoData;

  /// \brief A component that identifies an entity as being a Usv 
  using Usv = ignition::gazebo::components::Component<NoData, class UsvTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("mbzirc_components.Usv", Usv)

}  // namespace components
}  // namespace mbzirc

#endif  // MBZIRC_IGN__COMPONENTS_HH_


