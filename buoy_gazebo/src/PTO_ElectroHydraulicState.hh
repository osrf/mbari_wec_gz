/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_VBUS_HH_
#define IGNITION_GAZEBO_COMPONENTS_VBUS_HH_

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
  short Status; //should be 16 bits...
  
};


namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A volume component where the units are m^3.
  /// Double value indicates volume of an entity.
  using PTO_State = Component<ElectroHydraulicState, class ElectroHydraulicStateTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.PTO_State", PTO_State)
}
}
}
}

#endif

