// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "PTOFriction.hpp"
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>

#include <stdio.h>

#include <simple_interp/interp1d.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>


namespace buoy_gazebo
{
class PTOFrictionPrivate
{
public:
  /// \brief Piston joint entity
  ignition::gazebo::Entity PrismaticJointEntity{ignition::gazebo::kNullEntity};

  /// \brief Model interface
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  /// \brief Piston velocity
  const std::vector<double> pistonSpeed;  // m/s

  /// \brief Piston mean friction force
  const std::vector<double> meanFriction;  // N

  /// \brief Construct and approximation of friction model using linear spline
  simple_interp::Interp1d pto_friction_model;

  PTOFrictionPrivate()
  : pistonSpeed{-5.0, -0.4, -0.1, -0.05, -0.01, 0.0,
      0.01, 0.05, 0.1, 0.4, 5.0},
    meanFriction{12750.0, 1200.0, 700.0, 400.0, 160.0, 0.0,
      -550.0, -750.0, -1000.0, -2900.0, -32033.0},
    pto_friction_model(pistonSpeed, meanFriction)
  {
  }
};

//////////////////////////////////////////////////
PTOFriction::PTOFriction()
: dataPtr(std::make_unique<PTOFrictionPrivate>())
{
}

//////////////////////////////////////////////////
void PTOFriction::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "PTOFriction plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }


  // Get params from SDF for Prismatic Joint.
  auto PrismaticJointName = _sdf->Get<std::string>("PrismaticJointName");
  if (PrismaticJointName.empty()) {
    ignerr << "PTOFriction found an empty PrismaticJointName parameter. " <<
      "Failed to initialize.";
    return;
  }


  this->dataPtr->PrismaticJointEntity = this->dataPtr->model.JointByName(
    _ecm,
    PrismaticJointName);
  if (this->dataPtr->PrismaticJointEntity == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name [" << PrismaticJointName << "] not found. " <<
      "The PTOFriction may not influence this joint.\n";
    return;
  }
}

//////////////////////////////////////////////////
void PTOFriction::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("PTOFriction::PreUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  auto SimTime = std::chrono::duration<double>(_info.simTime).count();

  // If the joints haven't been identified yet, the plugin is disabled
  if (this->dataPtr->PrismaticJointEntity == ignition::gazebo::kNullEntity) {
    return;
  }

  // Create joint velocity component for piston if one doesn't exist
  auto prismaticJointVelComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(
    this->dataPtr->PrismaticJointEntity);
  if (prismaticJointVelComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->PrismaticJointEntity, ignition::gazebo::components::JointVelocity());
  }
  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (prismaticJointVelComp == nullptr || prismaticJointVelComp->Data().empty()) {
    return;
  }

  // Interpolate the new friction force based on current joint velocity
  // Velocity and Force sign flipped to account for the direction difference between
  // sim and physical buoy
  double friction_force =
    -this->dataPtr->pto_friction_model.eval(
    -prismaticJointVelComp->Data().at(0));

  // Create new component for applying force if it doesn't already exist
  auto forceComp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(
    this->dataPtr->PrismaticJointEntity);
  if (forceComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->PrismaticJointEntity,
      ignition::gazebo::components::JointForceCmd({friction_force}));  // Create this iteration
  } else {
    forceComp->Data()[0] += friction_force;  // Add friction to existing forces
  }
}

}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::PTOFriction,
  ignition::gazebo::System,
  buoy_gazebo::PTOFriction::ISystemConfigure,
  buoy_gazebo::PTOFriction::ISystemPreUpdate);
