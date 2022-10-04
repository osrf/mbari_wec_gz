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

#include "HydraulicPneumaticFriction.hpp"
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>

#include <stdio.h>

#include <splinter_ros/splinter1d.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>


namespace buoy_gazebo
{
class HydraulicPneumaticFrictionPrivate
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
  splinter_ros::Splinter1d hydraulic_pneumatic_friction;

  HydraulicPneumaticFrictionPrivate()
  : pistonSpeed{-0.4F, -0.3F, -0.2F, -0.1F, 0.0F, 0.1F, 0.2F, 0.3F, 0.4F},
    meanFriction{1200.0F, 1000.0F, 700.0F, 500.0F, 0.0F, -1000.0F, -1400.0F, -2100.0F, -2900.0F},
    hydraulic_pneumatic_friction(pistonSpeed, meanFriction)
  {
  }
};

//////////////////////////////////////////////////
HydraulicPneumaticFriction::HydraulicPneumaticFriction()
: dataPtr(std::make_unique<HydraulicPneumaticFrictionPrivate>())
{
}

//////////////////////////////////////////////////
void HydraulicPneumaticFriction::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "HydraulicPneumaticFriction plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }


  // Get params from SDF for Prismatic Joint.
  auto PrismaticJointName = _sdf->Get<std::string>("PrismaticJointName");
  if (PrismaticJointName.empty()) {
    ignerr << "HydraulicPneumaticFriction found an empty PrismaticJointName parameter. " <<
      "Failed to initialize.";
    return;
  }


  this->dataPtr->PrismaticJointEntity = this->dataPtr->model.JointByName(
    _ecm,
    PrismaticJointName);
  if (this->dataPtr->PrismaticJointEntity == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name [" << PrismaticJointName << "] not found. " <<
      "The HydraulicPneumaticFriction may not influence this joint.\n";
    return;
  }
}

//////////////////////////////////////////////////
void HydraulicPneumaticFriction::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("HydraulicPneumaticFriction::PreUpdate");
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
  auto friction_force =
    this->dataPtr->hydraulic_pneumatic_friction.eval(fabs(prismaticJointVelComp->Data().at(0)));

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
  buoy_gazebo::HydraulicPneumaticFriction,
  ignition::gazebo::System,
  buoy_gazebo::HydraulicPneumaticFriction::ISystemConfigure,
  buoy_gazebo::HydraulicPneumaticFriction::ISystemPreUpdate);
