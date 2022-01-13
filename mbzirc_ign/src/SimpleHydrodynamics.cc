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

#include <string>
#include <Eigen/Eigen>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "SimpleHydrodynamics.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::SimpleHydrodynamicsPrivate
{
  /// \brief The link entity.
  public: ignition::gazebo::Link link;

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Added mass in surge, X_\dot{u}.
  public: double paramXdotU;

  /// \brief Added mass in sway, Y_\dot{v}.
  public: double paramYdotV;

  /// \brief Added mass in heave, Z_\dot{w}.
  public: double paramZdotW;

  /// \brief Added mass in roll, K_\dot{p}.
  public: double paramKdotP;

  /// \brief Added mass in pitch, M_\dot{q}.
  public: double paramMdotQ;

  /// \brief Added mass in yaw, N_\dot{r}.
  public: double paramNdotR;

  /// \brief Linear drag in surge.
  public: double paramXu;

  /// \brief Quadratic drag in surge.
  public: double paramXuu;

  /// \brief Linear drag in sway.
  public: double paramYv;

  /// \brief Quadratic drag in sway.
  public: double paramYvv;

  /// \brief Linear drag in heave.
  public: double paramZw;

  /// \brief Quadratic drag in heave.
  public: double paramZww;

  /// \brief Linear drag in roll.
  public: double paramKp;

  /// \brief Quadratic drag in roll.
  public: double paramKpp;

  /// \brief Linear drag in pitch.
  public: double paramMq;

  /// \brief Quadratic drag in pitch.
  public: double paramMqq;

  /// \brief Linear drag in yaw.
  public: double paramNr;

  /// \brief Quadratic drag in yaw.
  public: double paramNrr;

  /// \brief Added mass of vehicle.
  /// See: https://en.wikipedia.org/wiki/Added_mass
  public: Eigen::MatrixXd Ma;
};


//////////////////////////////////////////////////
SimpleHydrodynamics::SimpleHydrodynamics()
  : dataPtr(std::make_unique<SimpleHydrodynamicsPrivate>())
{
}

//////////////////////////////////////////////////
void SimpleHydrodynamics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "No <link_name> specified" << std::endl;
    return;
  }

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    ignerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  this->dataPtr->link.EnableVelocityChecks(_ecm);
  this->dataPtr->link.EnableAccelerationChecks(_ecm);

  this->dataPtr->paramXdotU       = _sdf->Get<double>("xDotU", 5  ).first;
  this->dataPtr->paramYdotV       = _sdf->Get<double>("yDotV", 5  ).first;
  this->dataPtr->paramZdotW       = _sdf->Get<double>("zDotW", 0.1).first;
  this->dataPtr->paramKdotP       = _sdf->Get<double>("kDotP", 0.1).first;
  this->dataPtr->paramMdotQ       = _sdf->Get<double>("mDotQ", 0.1).first;
  this->dataPtr->paramNdotR       = _sdf->Get<double>("nDotR", 1  ).first;
  this->dataPtr->paramXu          = _sdf->Get<double>("xU",   20  ).first;
  this->dataPtr->paramXuu         = _sdf->Get<double>("xUU",   0  ).first;
  this->dataPtr->paramYv          = _sdf->Get<double>("yV",   20  ).first;
  this->dataPtr->paramYvv         = _sdf->Get<double>("yVV",   0  ).first;
  this->dataPtr->paramZw          = _sdf->Get<double>("zW",   20  ).first;
  this->dataPtr->paramZww         = _sdf->Get<double>("zWW",   0  ).first;
  this->dataPtr->paramKp          = _sdf->Get<double>("kP",   20  ).first;
  this->dataPtr->paramKpp         = _sdf->Get<double>("kPP",   0  ).first;
  this->dataPtr->paramMq          = _sdf->Get<double>("mQ",   20  ).first;
  this->dataPtr->paramMqq         = _sdf->Get<double>("mQQ",   0  ).first;
  this->dataPtr->paramNr          = _sdf->Get<double>("nR",   20  ).first;
  this->dataPtr->paramNrr         = _sdf->Get<double>("nRR",   0  ).first;

  // Added mass according to Fossen's equations (p 37).
  this->dataPtr->Ma = Eigen::MatrixXd::Zero(6, 6);

  this->dataPtr->Ma(0, 0) = this->dataPtr->paramXdotU;
  this->dataPtr->Ma(1, 1) = this->dataPtr->paramYdotV;
  this->dataPtr->Ma(2, 2) = this->dataPtr->paramZdotW;
  this->dataPtr->Ma(3, 3) = this->dataPtr->paramKdotP;
  this->dataPtr->Ma(4, 4) = this->dataPtr->paramMdotQ;
  this->dataPtr->Ma(5, 5) = this->dataPtr->paramNdotR;

  igndbg << "SimpleHydrodynamics plugin successfully configured with the "
         << "following parameters:"                        << std::endl;
  igndbg << "  <link_name>: " << linkName                  << std::endl;
  igndbg << "  <xDotU>: "     << this->dataPtr->paramXdotU << std::endl;
  igndbg << "  <yDotV>: "     << this->dataPtr->paramYdotV << std::endl;
  igndbg << "  <zDotW>: "     << this->dataPtr->paramZdotW << std::endl;
  igndbg << "  <kDotP>: "     << this->dataPtr->paramKdotP << std::endl;
  igndbg << "  <mDotQ>: "     << this->dataPtr->paramMdotQ << std::endl;
  igndbg << "  <nDotR>: "     << this->dataPtr->paramNdotR << std::endl;
  igndbg << "  <xU>: "        << this->dataPtr->paramXu    << std::endl;
  igndbg << "  <xUU>: "       << this->dataPtr->paramXuu   << std::endl;
  igndbg << "  <yV>: "        << this->dataPtr->paramYv    << std::endl;
  igndbg << "  <yVV>: "       << this->dataPtr->paramYvv   << std::endl;
  igndbg << "  <zW>: "        << this->dataPtr->paramZw    << std::endl;
  igndbg << "  <zWW>: "       << this->dataPtr->paramZww   << std::endl;
  igndbg << "  <kP>: "        << this->dataPtr->paramKp    << std::endl;
  igndbg << "  <kPP>: "       << this->dataPtr->paramKpp   << std::endl;
  igndbg << "  <mQ>: "        << this->dataPtr->paramMq    << std::endl;
  igndbg << "  <mQQ>: "       << this->dataPtr->paramMqq   << std::endl;
  igndbg << "  <nR>: "        << this->dataPtr->paramNr    << std::endl;
  igndbg << "  <nRR>: "       << this->dataPtr->paramNrr   << std::endl;
}

//////////////////////////////////////////////////
void SimpleHydrodynamics::PreUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("SimpleHydrodynamics::PreUpdate");

  if (!this->dataPtr->link.Valid(_ecm))
    return;

  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);

  // Get vehicle state.
  auto angularVel = this->dataPtr->link.WorldAngularVelocity(_ecm);
  auto linearVel = this->dataPtr->link.WorldLinearVelocity(_ecm);
  auto angularAccel = this->dataPtr->link.WorldAngularAcceleration(_ecm);
  auto linearAccel = this->dataPtr->link.WorldLinearAcceleration(_ecm);

  if (!angularVel)
  {
    ignerr << "No angular velocity" <<"\n";
    return;
  }

  if (!linearVel)
  {
    ignerr << "No linear velocity" <<"\n";
    return;
  }

  if (!angularAccel)
  {
    ignerr << "No angular acceleration" <<"\n";
    return;
  }

  if (!linearAccel)
  {
    ignerr << "No linear acceleration" <<"\n";
    return;
  }

  stateDot << (*linearAccel).X(), (*linearAccel).Y(), (*linearAccel).Z(),
   (*angularAccel).X(), (*angularAccel).Y(), (*angularAccel).Z();

  state << (*linearVel).X(), (*linearVel).Y(), (*linearVel).Z(),
    (*angularVel).X(), (*angularVel).Y(), (*angularVel).Z();

  // Added Mass.
  const Eigen::VectorXd kAmassVec = -1.0 * this->dataPtr->Ma * stateDot;

  // Coriolis - added mass components.
  Cmat(0, 5) = this->dataPtr->paramYdotV * (*linearVel).Y();
  Cmat(1, 5) = this->dataPtr->paramXdotU * (*linearVel).X();
  Cmat(5, 0) = this->dataPtr->paramYdotV * (*linearVel).Y();
  Cmat(5, 1) = this->dataPtr->paramXdotU * (*linearVel).X();

  // Drag.
  Dmat(0, 0) = this->dataPtr->paramXu +
    this->dataPtr->paramXuu * std::abs((*linearVel).X());
  Dmat(1, 1) = this->dataPtr->paramYv +
    this->dataPtr->paramYvv * std::abs((*linearVel).Y());
  Dmat(2, 2) = this->dataPtr->paramZw +
    this->dataPtr->paramZww * std::abs((*linearVel).Z());
  Dmat(3, 3) = this->dataPtr->paramKp +
    this->dataPtr->paramKpp * std::abs((*angularVel).X());
  Dmat(4, 4) = this->dataPtr->paramMq +
    this->dataPtr->paramMqq * std::abs((*angularVel).Y());
  Dmat(5, 5) = this->dataPtr->paramNr +
    this->dataPtr->paramNrr * std::abs((*angularVel).Z());

  const Eigen::VectorXd kDvec = -1.0 * Dmat * state;

  // Sum all forces - in body frame.
  const Eigen::VectorXd kForceSum = kAmassVec + kDvec;

  // Transform the force and torque to the world frame.
  ignition::math::Vector3d forceWorld =
    (*this->dataPtr->link.WorldInertialPose(_ecm)).Rot().RotateVector(
      ignition::math::Vector3d(kForceSum(0), kForceSum(1), kForceSum(2)));
  ignition::math::Vector3d torqueWorld =
    (*this->dataPtr->link.WorldInertialPose(_ecm)).Rot().RotateVector(
      ignition::math::Vector3d(kForceSum(3), kForceSum(4), kForceSum(5)));

  // Apply the force and torque at COM.
  this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
}

IGNITION_ADD_PLUGIN(SimpleHydrodynamics,
                    ignition::gazebo::System,
                    SimpleHydrodynamics::ISystemConfigure,
                    SimpleHydrodynamics::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SimpleHydrodynamics,
                          "ignition::gazebo::systems::SimpleHydrodynamics")
