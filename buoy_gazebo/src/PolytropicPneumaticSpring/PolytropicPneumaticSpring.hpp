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

namespace buoy_gazebo
{
// enum class SpringType { linear, pneumatic_adiabatic, pneumatic_calibrated};

// Forward declaration
struct PolytropicPneumaticSpringPrivate;

/// \brief This can be attached to a model with a reference
/// to a single prismatic joint. A force proportional to the
/// joint displacement will be applied along the axis of the joint.
/// This is an added force that will sum to other forces that may be present.
///
/// ## System Parameters
///
/// xml tags in Ignition Gazebo .sdf file define behavior as follows:
///
/// \brief <JointName>  The name of the joint to control. Required parameter.
///
/// <SpringType> \brief Type of Spring, options are 'linear', 'pneumatic_adiabatic',
///               'pneumatic_calibrated'  - Currently Unused
///
/// <SpringConst> \brief The spring constant.
///                Required and used when 'SpringType' is 'trivial'.
///                The default is 1.
///
/// <PistonDiam> \brief Piston Diam (inches)
///               Required and used whenever 'SpringType' is not 'trivial'.
///               The default is 5.  - Currently Unused
///
/// <RodDiam> \brief Rod Diamter (inches)
///            Required and used whenever 'SpringType' is not 'trivial'.
///            The default is 1.5.  - Currently Unused
///
/// <PistonEndVolume> \brief Piston End Dead Volume when position is 0 (inches^3).
///                    Required and used whenever 'SpringType' is not 'trivial'.
///                    The default is 1430.  - Currently Unused
///
/// <RodEndVolume> \brief Rod End Dead Volume when position is 0 (inches^3).
///                 Required and used whenever 'SpringType' is not 'trivial'.
///                 The default is 4700.  - Currently Unused
///
/// <PistonEndPressure> \brief Piston End pressure when position is 0 (psia).
///                      Required and used whenever 'SpringType' is not 'trivial'.
///                      The default is 65.  - Currently Unused
///
/// <RodEndPressure> \brief Rod End pressure when position is 0 (psia).
///                   Required and used whenever 'SpringType' is not 'trivial'.
///                   The default is 160.  - Currently Unused
///
/// <AmbientTemp> \brief Ambient Temperature (degrees C).
///                Required and used whenever 'SpringType' is not 'trivial'.
///                The default is 15.  - Currently Unused
///

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
  void openValve(const int dt_nano,
    double & P0, double & V0);
  void openValve(const int dt_nano,
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
