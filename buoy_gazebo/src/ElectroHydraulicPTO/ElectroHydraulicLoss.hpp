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

#ifndef ELECTROHYDRAULICPTO__ELECTROHYDRAULICLOSS_HPP_
#define ELECTROHYDRAULICPTO__ELECTROHYDRAULICLOSS_HPP_


#include <cmath>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>


namespace buoy_gazebo
{

/// \brief State data for power commands and feedback from sensors for PCRecord message in ROS2
struct ElectroHydraulicLoss
{
  float hydraulic_motor_loss{0.0F};
  float relief_valve_loss{0.0F};
  float motor_drive_i2r_loss{0.0F};
  float motor_drive_switching_loss{0.0F};
  float motor_drive_friction_loss{0.0F};
  float battery_i2r_loss{0.0F};

  bool operator==(const ElectroHydraulicLoss & that) const
  {
    bool equal = fabs(this->hydraulic_motor_loss - that.hydraulic_motor_loss) < 1e-7F;
    equal &= fabs(this->relief_valve_loss - that.relief_valve_loss) < 1e-7F;
    equal &= fabs(this->motor_drive_i2r_loss - that.motor_drive_i2r_loss) < 1e-7F;
    equal &= fabs(this->motor_drive_switching_loss - that.motor_drive_switching_loss) < 1e-7F;
    equal &= fabs(this->motor_drive_friction_loss - that.motor_drive_friction_loss) < 1e-7F;
    equal &= fabs(this->battery_i2r_loss - that.battery_i2r_loss) < 1e-7F;
    return equal;
  }
};

namespace components
{
/// \brief State data as component for reporting via ROS2
using ElectroHydraulicLoss =
  gz::sim::components::Component<buoy_gazebo::ElectroHydraulicLoss,
    class ElectroHydraulicLossTag>;
GZ_SIM_REGISTER_COMPONENT(
  "buoy_gazebo.components.ElectroHydraulicLoss",
  ElectroHydraulicLoss)
}  // namespace components

}  // namespace buoy_gazebo

#endif  // ELECTROHYDRAULICPTO__ELECTROHYDRAULICLOSS_HPP_
