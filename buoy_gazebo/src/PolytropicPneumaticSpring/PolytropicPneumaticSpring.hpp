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

#ifndef POLYTROPICPNEUMATICSPRING__POLYTROPICPNEUMATICSPRING_HPP_
#define POLYTROPICPNEUMATICSPRING__POLYTROPICPNEUMATICSPRING_HPP_

#include <ignition/gazebo/System.hh>
#include <ignition/transport.hh>

#include <memory>

#include "SpringState.hpp"


namespace buoy_gazebo
{
// Forward declaration
struct PolytropicPneumaticSpringPrivate;

/// TODO(andermi) documentation
/// SDF parameters:
/// * `<>`: 
class PolytropicPneumaticSpring : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  /// \brief Constructor
  PolytropicPneumaticSpring();

  /// \brief Destructor
  ~PolytropicPneumaticSpring() override = default;

  // Documentation inherited
  void Configure(
    const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

private:
  void openValve(
    const int dt_nano, const double & pressure_diff,
    double & P0, double & V0);
  void openValve(
    const int dt_nano, const double & pressure_diff,
    double & P1, double & V1,
    double & P2, double & V2);
  void computeForce(const double & x, const double & v, const double & n);

  ignition::transport::Node node;
  ignition::transport::Node::Publisher force_pub, pressure_pub, volume_pub,
    temperature_pub, heat_rate_pub, piston_velocity_pub;

  /// \brief Private data pointer
  std::unique_ptr<PolytropicPneumaticSpringPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // POLYTROPICPNEUMATICSPRING__POLYTROPICPNEUMATICSPRING_HPP_
