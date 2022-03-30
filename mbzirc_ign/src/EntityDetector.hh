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

#ifndef MBZIRC_IGN_ENTITYDETECTOR_HH_
#define MBZIRC_IGN_ENTITYDETECTOR_HH_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"

namespace mbzirc
{
  /// \brief A system system that publishes on a topic when an entity enters
  /// or leaves a specified region.
  ///
  /// An entity is detected when an entity's position enters the
  /// EntityDetector's region, which is represented by an
  /// ignition::math::AxisAlignedBox. When an entity is detected, the system
  /// publishes an ignition.msgs.Pose message with the pose of the detected
  /// entity with respect to the model containing the EntityDetector. The
  /// name and id fields of the Pose message will be set to the name and the
  /// entity of the detected entity respectively. The header of the Pose
  /// message contains the time stamp of detection. The `data` field of the
  /// header will contain the key "frame_id" with a value set to the name of
  /// the model containing the EntityDetector system and the key "state" with
  /// a value set to "1" if the entity is entering the detector's region and
  /// "0" if the entity is leaving the region. The `data` field of the
  /// header will also contain the key "count" with a value set to the
  /// number of entities currently in the region.
  ///
  /// The EntityDetector has to be attached to a `<model>` and it's region
  /// is centered on the containing model's origin.
  ///
  /// ## System parameters
  ///
  /// `<topic>`: Custom topic to be used for publishing when an entity is
  /// detected. If not set, the default topic with the following pattern would
  /// be used "/model/<model_name>/entity_detector/status". The topic type
  /// is ignition.msgs.Pose
  /// `<geometry>`: Detection region. Currently, only the `<box>` geometry is
  /// supported. The position of the geometry is derived from the pose of the
  /// containing model.
  /// `<pose>`: Additional pose offset relative to the parent model's pose.
  /// This pose is added to the parent model pose when computing the
  /// detection region. Only the position component of the `<pose>` is used.
  /// `<entities>`
  ///     `<name>`: Name of entity to detector

  class EntityDetector
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemPostUpdate
  {
    /// Documentation inherited
    public: EntityDetector() = default;

    /// Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Check if the entity has already been detected
    /// \param [in] _entity The entity to test
    /// \returns True if the entity has already been detected
    private: bool IsAlreadyDetected(const ignition::gazebo::Entity &_entity)
        const;

    /// \brief Add the entity to the list of detected entities
    /// \param [in] _entity The entity to add
    private: void AddToDetected(const ignition::gazebo::Entity &_entity);

    /// \brief Remove the entity from the list of detected entities
    /// \param [in] _entity The entity to remove
    private: void RemoveFromDetected(const ignition::gazebo::Entity &_entity);

    /// \brief Publish the event that the entity is detected or no longer
    /// detected.
    /// \param [in] _entity The entity to report
    /// \param [in] _name The name of the entity that triggered the event
    /// \param [in] _state The new state of the detector
    /// \param [in] _pose The pose of the entity that triggered the event
    /// \param [in] _stamp Time stamp of the event
    private: void Publish(const ignition::gazebo::Entity &_entity,
                          const std::string &_name,
                          bool _state, const ignition::math::Pose3d &_pose,
                          const std::chrono::steady_clock::duration &_stamp);

    /// \brief Keeps a set of detected entities
    private: std::unordered_set<ignition::gazebo::Entity> detectedEntities;

    /// \brief The model associated with this system.
    private: ignition::gazebo::Model model;

    /// \brief Name of the detector used as the frame_id in published messages.
    private: std::string detectorName;

    /// \brief Detector region. Only a box geometry is supported
    private: ignition::math::AxisAlignedBox detectorGeometry;

    /// \brief Ignition communication publisher.
    private: ignition::transport::Node::Publisher pub;

    /// \brief Whether the system has been initialized
    private: bool initialized{false};

    /// \brief Additional pose offset for the plugin.
    private: ignition::math::Pose3d poseOffset;

    /// \brief if world pose component has been enabled or not
    private: bool worldPoseEnabled{false};
  };
}

#endif
