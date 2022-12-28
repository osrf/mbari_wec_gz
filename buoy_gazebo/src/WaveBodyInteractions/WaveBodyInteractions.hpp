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

#ifndef WAVEBODYINTERACTIONS__WAVEBODYINTERACTIONS_HPP_
#define WAVEBODYINTERACTIONS__WAVEBODYINTERACTIONS_HPP_

#include <optional>

#include <gz/sim/System.hh>
#include <gz/transport.hh>
#include <memory>

namespace buoy_gazebo
{
// Forward declaration
class WaveBodyInteractionsPrivate;

/// \brief To use, several parameters are required.
/// Two gazebo sim joints that are either prismatic or continous with 1DOF each,
/// a desciption of the connectoins between actuator (joints),
/// and an oil characteristic specification.
///
/// ## System Parameters
///
/// xml tags in Ignition Gazebo .sdf file define behavior as follows:
///
/// \brief <PrismaticJointName>
///
///  For each actuator of prismatic type, the following nested tags are required:
///     <Area_A>  Piston area on A end of cylinder
///
///     <Area_B>  Piston area on B end of Cylinder
///
///
/// \brief <RevoluteJointName>
///   For each actuator of revolute type, the following nested tags are required:
///
///     <Displacement>  Displacement per revolution of rotary pump/motor.
class WaveBodyInteractions : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  /// \brief Constructor
  WaveBodyInteractions();

  /// \brief Destructor
  ~WaveBodyInteractions() override = default;

  // Documentation inherited
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  // Documentation inherited
  void Update(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  // Documentation inherited
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher pistonvel_pub;

  /// \brief Private data pointer
  std::unique_ptr<WaveBodyInteractionsPrivate> dataPtr;
};
}  // namespace buoy_gazebo
#endif  // WAVEBODYINTERACTIONS__WAVEBODYINTERACTIONS_HPP_
