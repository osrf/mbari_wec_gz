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

#include <buoy_utils/CommandTriState.hpp>
#include <buoy_utils/Status.hpp>


namespace buoy_gazebo
{
struct PowerStatusBits {
  uint8_t Mode : 2;  // controller mode. specified by defines in config.h
                     // (GENERATOR_MODE, TORQUE_MODE)
  uint8_t TorqueCmdLimited : 1;  // defines what is limited the Torque Command
                                 // (0 = Not limited, 1 = Rate, 2 = Min, 3 = Max).
  uint8_t BattSwitchRequest : 1;  // Indicates state of battery switch that the user desires.
                                  // 0 = off, 1 = on
  uint8_t BattSwitchSetting : 1;  // Indicates instantaneous setting of Battery Switch
  uint8_t SlowSwitchSetting : 1;  // Indicates instantaneous setting of Battery Slow Switch
  uint8_t EXT_Voltage_Detect : 1;  // Indicates if >170V is available on outside connector
  uint8_t GainScheduleMode : 1;  // 0 = off. 1 = on.
  uint8_t OverCurrentShutdown : 1;  // 1 indicates drive is in overcurrent shutdown.
  uint8_t OverVoltageShutdown : 1;  // 1 indicates drive is in overvoltage shutdown.
  uint8_t SpringRangeValid : 1;  // 1 indicates the current SC_Range value is valid (received
                                 // within approximately SC_RANGE_VALID_TIMEOUT milliseconds)
  uint8_t PermissiveMode : 1;  // 1 indicates user has selected "permissive mode" which allows
                               // WindingCurrent commands to be set even if SpringController Range
                               // packets aren't arriving
};

/// \brief State data for power commands and feedback from sensors for PCRecord message in ROS2
struct ElectroHydraulicState
{
  float rpm{0.0F};
  float sd_rpm{0.0F};  // TODO(anyone) not set
  float voltage{0.0F};
  float draw_curr_limit{0.0F};  // TODO(anyone) not set
  float bcurrent{0.0F};
  float wcurrent{0.0F};
  float torque{0.0F};  // TODO(anyone) not set
  float diff_press{0.0F};
  float bias_current{0.0F};
  float loaddc{0.0F};
  float scale{0.0F};
  float retract{0.0F};
  float target_v{0.0F};  // TODO(anyone) not set
  float target_a{0.0F};
  buoy_utils::Status<PowerStatusBits> status;  // NOT CURRENTLY IMPLEMENTED
  float charge_curr_limit{0.0F};  // TODO(anyone) not set

  buoy_utils::CommandTriState<> torque_command;
  buoy_utils::CommandTriState<> scale_command;
  buoy_utils::CommandTriState<> retract_command;
  buoy_utils::CommandTriState<> bias_current_command;

  bool operator==(const ElectroHydraulicState & that) const
  {
    bool equal = fabs(this->rpm - that.rpm) < 1e-7F;
    equal &= fabs(this->sd_rpm - that.sd_rpm) < 1e-7F;
    equal &= fabs(this->voltage - that.voltage) < 1e-7F;
    equal &= fabs(this->draw_curr_limit - that.draw_curr_limit) < 1e-7F;
    equal &= fabs(this->bcurrent - that.bcurrent) < 1e-7F;
    equal &= fabs(this->wcurrent - that.wcurrent) < 1e-7F;
    equal &= fabs(this->torque - that.torque) < 1e-7F;
    equal &= fabs(this->diff_press - that.diff_press) < 1e-7F;
    equal &= fabs(this->bias_current - that.bias_current) < 1e-7F;
    equal &= fabs(this->loaddc - that.loaddc) < 1e-7F;
    equal &= fabs(this->scale - that.scale) < 1e-7F;
    equal &= fabs(this->retract - that.retract) < 1e-7F;
    equal &= fabs(this->target_v - that.target_v) < 1e-7F;
    equal &= fabs(this->target_a - that.target_a) < 1e-7F;
    equal &= this->status == that.status;
    equal &= fabs(this->charge_curr_limit - that.charge_curr_limit) < 1e-7F;
    return equal;
  }
};

namespace components
{
/// \brief State data as component for power commands and feedback from sensors for PCRecord
/// message in ROS2
using ElectroHydraulicState =
  ignition::gazebo::components::Component<buoy_gazebo::ElectroHydraulicState,
    class ElectroHydraulicStateTag>;
IGN_GAZEBO_REGISTER_COMPONENT(
  "buoy_gazebo.components.ElectroHydraulicState",
  ElectroHydraulicState)
}  // namespace components

}  // namespace buoy_gazebo

#endif  // ELECTROHYDRAULICPTO__ELECTROHYDRAULICSTATE_HPP_
