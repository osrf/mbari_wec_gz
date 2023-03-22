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

#include <memory>

#include <gz/sim/System.hh>
#include <gz/transport.hh>

namespace buoy_gazebo
{
// Forward declaration
struct PolytropicPneumaticSpringPrivate;

/// \brief Models one chamber of air spring system in buoy.
/// An upper and lower chamber should be attached to piston joint to form the spring.
/// This system plugin uses a polytropic relationship between Pressure and Volume: PV^n = constant.
/// `n` (polytropic index) may be equal to the adiabatic index in which case the system is
/// adiabatic and no heat loss occurs from the gas to the environment due to compression.
/// Hysteresis may also be represented in the system (different polytropic relationship depending)
/// on piston direction of travel.

/// SDF parameters:
/// * `<JointName>`: joint the plugin is attached to
/// * `<chamber>`: name of chamber
/// * `<is_upper>`: is this the upper chamber?
/// * `<valve_absement>`: measure of valve opening cross-section and duration (meter-seconds)
/// * `<pump_absement>`: measure of pump opening cross-section and duration (meter-seconds)
/// * `<pump_pressure>`: pump differential pressure
/// * `<stroke>`: piston stroke (m)
/// * `<piston_area>`: piston area (m^3)
/// * `<dead_volume>`: Piston-End Dead Volume (m^3)
/// * `<T0>`: initial Temp (K)
/// * `<R_specific>`: R (specific gas) constant
/// * `<c_p>`: specific heat of gas under constant pressure (kJ/(kg K))
/// * `<hysteresis>`: is hysteresis present in piston travel direction
/// * `<n>`: polytropic index
/// * `<n1>`: polytropic index for increasing volume (if hysteresis)
/// * `<n2>`: polytropic index for decreasing volume (if hysteresis)
/// * `<x0>`: initial piston position (m)
/// * `<x1>`: initial piston position (m) for increasing volume (if hysteresis)
/// * `<x2>`: initial piston position (m) for decreasing volume (if hysteresis)
/// * `<P0>`: initial Pressure (Pa)
/// * `<P1>`: initial Pressure (Pa) for increasing volume (if hysteresis)
/// * `<P2>`: initial Pressure (Pa) for decreasing volume (if hysteresis)
/// * `<debug_prescribed_velocity>`: will force joint to move sinusoidally for debugging

class PolytropicPneumaticSpring : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  PolytropicPneumaticSpring();

  /// \brief Destructor
  ~PolytropicPneumaticSpring() override = default;

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

private:
  /// \brief open valve to vent gas from lower to upper chamber
  void openValve(
    const int dt_nano, const double & pressure_diff,
    double & P0, const double & V0);
  /// \brief pump gas from upper to lower chamber
  void pumpOn(
    const int dt_nano,
    double & P0, const double & V0);
  void computeLawOfCoolingForce(const double & x, const int & dt_nano);
  void computePolytropicForce(const double & x, const double & v);

  gz::transport::Node node;
  gz::transport::Node::Publisher force_pub, pressure_pub, volume_pub,
    temperature_pub, heat_rate_pub, piston_velocity_pub;

  /// \brief Private data pointer
  std::unique_ptr<PolytropicPneumaticSpringPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // POLYTROPICPNEUMATICSPRING__POLYTROPICPNEUMATICSPRING_HPP_
