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

#ifndef POLYTROPICPNEUMATICSPRING__SPRINGSTATE_HPP_
#define POLYTROPICPNEUMATICSPRING__SPRINGSTATE_HPP_

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

#include <buoy_utils/CommandTriState.hpp>

namespace buoy_gazebo
{
/// \brief State data for spring commands and feedback from sensors for SCRecord message in ROS2
struct SpringState
{
  // SCRecord
  int16_t load_cell{0};  // load on Buoy->PTO universal joint in Newtons (TODO(andermi) units)
  float range_finder{0.0F};  // piston position in meters measured from fully retracted as
                             // reference. In buoy this is laser range finder at top of upper
                             // chamber (TODO(andermi) units)
  float upper_psi{0.0F};  // pressure in PSI (TODO(andermi) units)
  float lower_psi{0.0F};  // pressure in PSI (TODO(andermi) units)
  int16_t status{0};  // TODO(andermi) status bit field

  // Commands
  buoy_utils::CommandTriState valve_command;
  buoy_utils::CommandTriState pump_command;

  bool operator==(const SpringState & that) const
  {
    bool equal = this->load_cell == that.load_cell;
    equal &= fabs(this->range_finder - that.range_finder) < 1e-7;
    equal &= fabs(this->upper_psi - that.upper_psi) < 1e-7;
    equal &= fabs(this->lower_psi - that.lower_psi) < 1e-7;
    return equal;
  }
};

namespace components
{
using SpringState = ignition::gazebo::components::Component<buoy_gazebo::SpringState,
    class SpringStateTag>;
IGN_GAZEBO_REGISTER_COMPONENT("buoy_gazebo.components.SpringState", SpringState)
}  // namespace components

}  // namespace buoy_gazebo

#endif  // POLYTROPICPNEUMATICSPRING__SPRINGSTATE_HPP_
