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

#ifndef INCIDENTWAVES__INCWAVESTATE_HPP_
#define INCIDENTWAVES__INCWAVESTATE_HPP_

#include <memory>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>
#include <FreeSurfaceHydrodynamics/LinearIncidentWave.hpp>

namespace buoy_gazebo
{
/// \brief Structure that holds shared ptr to incident wave object(s), and other data
struct IncWaveState
{
  std::shared_ptr<LinearIncidentWave> Inc;
  double x;
  double y;
};

namespace components
{
/// \brief State
using IncWaveState =
  gz::sim::components::Component<buoy_gazebo::IncWaveState,
    class IncWaveStateTag>;
GZ_SIM_REGISTER_COMPONENT(
  "buoy_gazebo.components.IncWaveState",
  IncWaveState)
}  // namespace components

}  // namespace buoy_gazebo

#endif  // INCIDENTWAVES__INCWAVESTATE_HPP_
