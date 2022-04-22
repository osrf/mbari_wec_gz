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

#ifndef ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HH_
#define ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HH_

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

struct ElectroHydraulicState
{
  double xdot;
  double N;
  double deltaP;
  double VBus;
  double TargetWindingCurrent;
  double WindingCurrent;
  double I_Batt;
  double I_Load;
  double ScaleFactor;
  double RetractFactor;
  double BattChargeLimit;
  double BattDrawLimit;
  double RPMStdDev;
  double BiasCurrent;
  int16_t Status;  // should be 16 bits...
  ElectroHydraulicState() : xdot(0.0), N(0.0), deltaP(0.0), VBus(0.0),
    TargetWindingCurrent(0.0), WindingCurrent(0.0), I_Batt(0.0), I_Load(0.0),
    ScaleFactor(0.0), RetractFactor(0.0), BattChargeLimit(0.0),
    BattDrawLimit(0.0), RPMStdDev(0.0), BiasCurrent(0.0), Status(0) {}
};


namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace components
{
/// \brief A volume component where the units are m^3.
/// Double value indicates volume of an entity.
using PTO_State = Component<ElectroHydraulicState, class ElectroHydraulicStateTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.PTO_State", PTO_State)
}
}
}  // namespace gazebo
}  // namespace ignition

#endif  // ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HH_
