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

namespace buoy_gazebo
{

struct SpringState
{
  int16_t load_cell{0};
  float range_finder{0.0F};
  float upper_psi{0.0F};
  float lower_psi{0.0F};
};

using SpringStateComponent = ignition::gazebo::components::Component<SpringState,
    class SpringStateTag>;
IGN_GAZEBO_REGISTER_COMPONENT("buoy_gazebo.SpringState", SpringStateComponent)
}  // namespace buoy_gazebo

#endif  // POLYTROPICPNEUMATICSPRING__SPRINGSTATE_HPP_
