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

#include <ignition/common/Console.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>

#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/components/GpuLidar.hh>

#include <ignition/plugin/Register.hh>

#include <unordered_set>

#include "Components.hh"

using namespace ignition;

namespace mbzirc
{
  class CompetitorPlatformDetector:
    public gazebo::System,
    public gazebo::ISystemPreUpdate
  {
    public: std::unordered_set<gazebo::Entity> detectedPlatforms;

    public: void DetectModels(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<gazebo::components::Sensor,
                gazebo::components::ParentEntity>(
        [&](const gazebo::Entity &_entity,
            const gazebo::components::Sensor *,
            const gazebo::components::ParentEntity *_parent) -> bool
        {
          // Get the model. We are assuming that a sensor is attached to
          // a link.
          auto parent = _ecm.Component<gazebo::components::ParentEntity>(
              _parent->Data());
          auto model = parent;

          // find top level model
          while (parent && _ecm.Component<gazebo::components::Model>(
                 parent->Data()))
          {
            model = parent;
            parent = _ecm.Component<gazebo::components::ParentEntity>(
                parent->Data());
          }

          if (model)
          {
            gazebo::Entity entity = model->Data();

            // First time we have seen this model, populate info component
            if (!_ecm.EntityHasComponentType(entity, mbzirc::components::CompetitorPlatform().TypeId()))
            {
              auto name = _ecm.Component<gazebo::components::Name>(entity);
              auto pos = _ecm.Component<gazebo::components::Pose>(entity);
              auto info = _ecm.CreateComponent(entity, mbzirc::components::CompetitorPlatform());
              info->Data().modelEntity = entity;
              info->Data().robotName = name->Data();
              info->Data().initialPos = pos->Data().Pos();
              this->detectedPlatforms.insert(entity);
              this->EnumerateSensors(entity, _info, _ecm);
            }
          }
          return true;
        }
      );
    }

    public: void EnumerateSensors(const ignition::gazebo::Entity &_entity,
                                  const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto info = _ecm.Component<mbzirc::components::CompetitorPlatform>(_entity);
      _ecm.Each<gazebo::components::Sensor,
                gazebo::components::ParentEntity>(
        [&](const gazebo::Entity &_sensorEntity,
            const gazebo::components::Sensor *,
            const gazebo::components::ParentEntity *_parent) -> bool
        {
          std::string slot_name;
          auto parent = _ecm.Component<gazebo::components::ParentEntity>(
              _parent->Data());
          auto model = parent;

          // find top level model
          while (parent && _ecm.Component<gazebo::components::Model>(
                 parent->Data()))
          {
            auto name = _ecm.Component<gazebo::components::Name>(parent->Data());
            if (name->Data().find("sensor_") != std::string::npos)
            {
              slot_name = name->Data();
            }

            model = parent;
            parent = _ecm.Component<gazebo::components::ParentEntity>(
                parent->Data());
          }

          // Don't count sensors not in slots
          if (model && !slot_name.empty())
          {
            gazebo::Entity modelEntity = model->Data();

            // The sensor belongs to this platform
            if (modelEntity == _entity)
            {
              auto sensorInfo = mbzirc::SensorInfo();
              sensorInfo.sensorEntity = _sensorEntity;
              sensorInfo.slotName = slot_name;

              if (auto camera = _ecm.Component<gazebo::components::Camera>(_sensorEntity))
              {
                sensorInfo.sensorType = "camera";
              }
              else if (_ecm.Component<gazebo::components::RgbdCamera>(_sensorEntity))
              {
                sensorInfo.sensorType = "rgbd_camera";
              }
              else if (_ecm.Component<gazebo::components::GpuLidar>(_sensorEntity))
              {
                sensorInfo.sensorType = "gpu_lidar";
              }
              info->Data().sensors.push_back(sensorInfo);
            }
          }
          return true;
        });

      std::stringstream ss;
      ss << "Added New Competitor Platform: \n"
             << "  Name:       " << info->Data().robotName   << "\n"
             << "  Entity:     " << info->Data().modelEntity << "\n"
             << "  InitialPos: " << info->Data().initialPos   << "\n"
             << "  Sensors: \n";
      for (auto sensor_info: info->Data().sensors)
      {
        ss << "    Sensor Slot: " << sensor_info.slotName << "\n";
        ss << "    Sensor Type: " << sensor_info.sensorType << "\n";
      }
      igndbg << ss.str();
    }

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                           ignition::gazebo::EntityComponentManager &_ecm) final
    {
      this->DetectModels(_info, _ecm);
    }
  };
}  // namesapce mbzirc

IGNITION_ADD_PLUGIN(
    mbzirc::CompetitorPlatformDetector,
    ignition::gazebo::System,
    mbzirc::CompetitorPlatformDetector::ISystemPreUpdate)
