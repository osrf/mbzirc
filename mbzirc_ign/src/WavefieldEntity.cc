/*
 * Copyright (C) 2019  Rhys Mainwaring
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

#include <iostream>
#include <string>

#include <ignition/math/Vector2.hh>

#include "WavefieldEntity.hh"
#include "Wavefield.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private data for the WavefieldEntity
class ignition::gazebo::systems::WavefieldEntityPrivate
{
  /// \brief The size of the wavefield. Default value is [1000 1000].
  public: ignition::math::Vector2d size{1000, 1000};

  /// \brief The number of grid cells in the wavefield.
  /// Default value is [50 50].
  public: ignition::math::Vector2d cellCount{50, 50};

  /// \brief The wave parameters.
  public: std::shared_ptr<WaveParameters> waveParams;
};

/////////////////////////////////////////////////
WavefieldEntity::~WavefieldEntity()
{
}

/////////////////////////////////////////////////
WavefieldEntity::WavefieldEntity()
  : dataPtr(new WavefieldEntityPrivate())
{
}

/////////////////////////////////////////////////
void WavefieldEntity::Load(const std::shared_ptr<const sdf::Element> &_sdf)
{
  // Wavefield Parameters
  // this->dataPtr->size = _sdf->Get<ignition::math::Vector2d>("size",
  //   this->dataPtr->size).first;
  // this->dataPtr->cellCount = _sdf->Get<ignition::math::Vector2d>("cell_count",
  //   this->dataPtr->cellCount).first;

  // Wave Parameters
  ignmsg << "WavefieldEntity: Loading WaveParameters from SDF" <<  std::endl;
  this->dataPtr->waveParams.reset(new WaveParameters());
  // if (_sdf->HasElement("wave"))
  // {
  //   ignmsg << "Found <wave> tag" << std::endl;
  //   auto sdfWave = _sdf->GetElement("wave");
  //   this->dataPtr->waveParams->SetFromSDF(*sdfWave);
  // }
  // else
  // {
  //   ignmsg << "Missing <wave> tag" << std::endl;
  // }
  // @DEBUG_INFO
  this->dataPtr->waveParams->DebugPrint();
}

/////////////////////////////////////////////////
std::shared_ptr<WaveParameters> WavefieldEntity::Parameters()
{
  return this->dataPtr->waveParams;
}
