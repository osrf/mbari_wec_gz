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

#ifndef ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HPP_
#define ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HPP_

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

namespace buoy_gazebo
{

struct ElectroHydraulicState
{
  double xdot{0.0};
  double N{0.0};
  double deltaP{0.0};
  double VBus{0.0};
  double TargetWindingCurrent{0.0};
  double WindingCurrent{0.0};
  double I_Batt{0.0};
  double I_Load{0.0};
  double ScaleFactor{0.0};
  double RetractFactor{0.0};
  double BattChargeLimit{0.0};
  double BattDrawLimit{0.0};
  double RPMStdDev{0.0};
  double BiasCurrent{0.0};
  int16_t Status{0};
};

/// \brief A volume component where the units are m^3.
/// Double value indicates volume of an entity.
using PTO_State = ignition::gazebo::components::Component<ElectroHydraulicState,
    class ElectroHydraulicStateTag>;
IGN_GAZEBO_REGISTER_COMPONENT("buoy_gazebo.PTO_State", PTO_State)
}  // namespace buoy_gazebo

#endif  // ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HPP_
