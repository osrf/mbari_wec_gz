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

#ifndef ELECTROHYDRAULICPTO__BATTERYSTATE_HPP_
#define ELECTROHYDRAULICPTO__BATTERYSTATE_HPP_

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>


namespace buoy_gazebo
{
/// \brief State data for BCRecord message in ROS2
struct BatteryState
{
  float voltage{0.0F};
  float ips{0.0F};
  float vbalance{0.0F};
  float vstopcharge{0.0F};
  float gfault{0.0F};
  float hydrogen{0.0F};
  uint16_t status{0};
};

namespace components
{
/// \brief State data as component for BCRecord message in ROS2
using BatteryState =
  gz::sim::components::Component<buoy_gazebo::BatteryState,
    class BatteryStateTag>;
GZ_SIM_REGISTER_COMPONENT(
  "buoy_gazebo.components.BatteryState",
  BatteryState)
}  // namespace components

}  // namespace buoy_gazebo

#endif  // ELECTROHYDRAULICPTO__BATTERYSTATE_HPP_
