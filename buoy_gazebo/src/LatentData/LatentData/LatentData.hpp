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

#ifndef LATENTDATA__LATENTDATA__LATENTDATA_HPP_
#define LATENTDATA__LATENTDATA__LATENTDATA_HPP_

#include <vector>

#include <gz/math/Vector3.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>


namespace buoy_gazebo
{
struct IncWaveHeightPoint
{
  // ==== position ====
  // x, y may be relative to buoy origin
  bool use_buoy_origin{false};  // input
  double x{0.0};  // input
  double y{0.0};  // input

  // z (height)
  double eta{0.0};  // output

  // ==== orientation ====
  // normal vector of deta/dx, deta/dy (slope of wave)
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{0.0};  // output

  bool operator==(const IncWaveHeightPoint & that) const
  {
    bool equal = fabs(this->x - that.x) < 1e-7F;
    equal &= fabs(this->y - that.y) < 1e-7F;
    equal &= fabs(this->eta - that.eta) < 1e-7F;
    equal &= fabs(this->qx - that.qx) < 1e-7F;
    equal &= fabs(this->qy - that.qy) < 1e-7F;
    equal &= fabs(this->qz - that.qz) < 1e-7F;
    equal &= fabs(this->qw - that.qw) < 1e-7F;
    return equal;
  }
};

struct IncWaveHeights
{
  int32_t sec{0};
  uint32_t nsec{0U};
  bool valid;
  std::vector<IncWaveHeightPoint> points;

  bool operator==(const IncWaveHeights & that) const
  {
    // shortcut different sizes as not equal
    bool equal = (this->points.size() == that.points.size());
    equal &= this->valid == that.valid;
    if (!equal) {
      return false;
    }

    // since sizes are equal, iterate over all points (or none)
    std::size_t idx = 0U;
    for (; idx < this->points.size(); ++idx) {
      equal &= (this->points[idx] == that.points[idx]);
    }

    return equal;
  }
};

struct AirSpring
{
  bool valid{false};
  double force{0.0};  // Newtons
  double T{0.0};  // temp in K
  double dQ_dt{0.0};  // heat loss rate
  double piston_position{0.0};  // meters
  double piston_velocity{0.0};  // m/s
  double mass{0.0};  // kg

  bool operator==(const AirSpring & that) const
  {
    bool equal = (this->valid == that.valid);
    equal &= fabs(this->force - that.force) < 1e-7F;
    equal &= fabs(this->T - that.T) < 1e-7F;
    equal &= fabs(this->dQ_dt - that.dQ_dt) < 1e-7F;
    equal &= fabs(this->piston_position - that.piston_position) < 1e-7F;
    equal &= fabs(this->piston_velocity - that.piston_velocity) < 1e-7F;
    equal &= fabs(this->mass - that.mass) < 1e-7F;

    return equal;
  }
};

struct ElectroHydraulic
{
  bool valid{false};
  double rpm{0.0};
  double upper_hydraulic_pressure{0.0};  //  Pa
  double lower_hydraulic_pressure{0.0};  //  Pa
  double force{0.0};  // Newtons
  double supplied_hydraulic_power{0.0};  // Watts
  double hydraulic_motor_loss{0.0};  // Watts
  double relief_valve_loss{0.0};  // Watts
  double shaft_mech_power{0.0};  // Watts
  double motor_drive_i2r_loss{0.0};  // Watts
  double motor_drive_switching_loss{0.0};  // Watts
  double motor_drive_friction_loss{0.0};  // Watts
  double load_dump_power{0.0};  // Watts
  double battery_i2r_loss{0.0};  // Watts
  double battery_storage_power{0.0};  // Watts

  bool operator==(const ElectroHydraulic & that) const
  {
    bool equal = (this->valid == that.valid);
    equal &= fabs(this->rpm - that.rpm) < 1e-7F;
    equal &= fabs(this->upper_hydraulic_pressure - that.upper_hydraulic_pressure) < 1e-7F;
    equal &= fabs(this->lower_hydraulic_pressure - that.lower_hydraulic_pressure) < 1e-7F;
    equal &= fabs(this->force - that.force) < 1e-7F;
    equal &= fabs(this->supplied_hydraulic_power - that.supplied_hydraulic_power) < 1e-7F;
    equal &= fabs(this->hydraulic_motor_loss - that.hydraulic_motor_loss) < 1e-7F;
    equal &= fabs(this->relief_valve_loss - that.relief_valve_loss) < 1e-7F;
    equal &= fabs(this->shaft_mech_power - that.shaft_mech_power) < 1e-7F;
    equal &= fabs(this->motor_drive_i2r_loss - that.motor_drive_i2r_loss) < 1e-7F;
    equal &= fabs(this->motor_drive_switching_loss - that.motor_drive_switching_loss) < 1e-7F;
    equal &= fabs(this->motor_drive_friction_loss - that.motor_drive_friction_loss) < 1e-7F;
    equal &= fabs(this->load_dump_power - that.load_dump_power) < 1e-7F;
    equal &= fabs(this->battery_i2r_loss - that.battery_i2r_loss) < 1e-7F;
    equal &= fabs(this->battery_storage_power - that.battery_storage_power) < 1e-7F;

    return equal;
  }
};

struct WaveBody
{
  bool valid{false};

  gz::math::Pose3<double> pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  gz::math::Pose3<double> twist{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  gz::math::Vector3d buoyant_force{0.0, 0.0, 0.0};
  gz::math::Vector3d buoyant_moment{0.0, 0.0, 0.0};
  double buoyancy_total_power{0.0};
  gz::math::Vector3d radiation_force{0.0, 0.0, 0.0};
  gz::math::Vector3d radiation_moment{0.0, 0.0, 0.0};
  double radiation_total_power{0.0};
  gz::math::Vector3d exciting_force{0.0, 0.0, 0.0};
  gz::math::Vector3d exciting_moment{0.0, 0.0, 0.0};
  double excitation_total_power{0.0};

  bool operator==(const WaveBody & that) const
  {
    bool equal = (this->valid == that.valid);
    equal &= (this->pose == that.pose);
    equal &= (this->twist == that.twist);
    equal &= (this->buoyant_force == that.buoyant_force);
    equal &= (this->buoyant_moment == that.buoyant_moment);
    equal &= fabs(this->buoyancy_total_power - that.buoyancy_total_power) < 1e-7F;
    equal &= (this->radiation_force == that.radiation_force);
    equal &= (this->radiation_moment == that.radiation_moment);
    equal &= fabs(this->radiation_total_power - that.radiation_total_power) < 1e-7F;
    equal &= (this->exciting_force == that.exciting_force);
    equal &= (this->exciting_moment == that.exciting_moment);
    equal &= fabs(this->excitation_total_power - that.excitation_total_power) < 1e-7F;

    return equal;
  }
};

/// \brief latent data that is modeled but not directly observed for LatentData message in ROS 2
struct LatentData
{
  IncWaveHeights inc_wave_heights;
  AirSpring upper_spring;
  AirSpring lower_spring;
  ElectroHydraulic electro_hydraulic;
  WaveBody wave_body;

  double piston_friction_force_valid;
  double piston_friction_force;  // Newtons

  bool valid() const
  {
    /* std::cout << "iwh[" << inc_wave_heights.valid << "] :: "
      << "us[" << upper_spring.valid << "] :: "
      << "ls[" << lower_spring.valid << "] :: "
      << "wb[" << wave_body.valid << "] :: "
      << "friction[" << piston_friction_force_valid << "]" << std::endl; */
    return inc_wave_heights.valid && \
           upper_spring.valid && lower_spring.valid && \
           electro_hydraulic.valid && wave_body.valid && \
           piston_friction_force_valid;
  }

  bool operator==(const LatentData & that) const
  {
    bool equal = (this->inc_wave_heights == that.inc_wave_heights);
    equal &= (this->upper_spring == that.upper_spring);
    equal &= (this->lower_spring == that.lower_spring);
    equal &= (this->electro_hydraulic == that.electro_hydraulic);
    equal &= (this->wave_body == that.wave_body);
    equal &= fabs(this->piston_friction_force - that.piston_friction_force) < 1e-7F;
    equal &= (this->piston_friction_force_valid == that.piston_friction_force_valid);
    return equal;
  }
};

namespace components
{
/// \brief Latent data as component for that is modeled but not directly observed for LatentData
/// message in ROS 2
using LatentData =
  gz::sim::components::Component<buoy_gazebo::LatentData,
    class LatentDataTag>;
GZ_SIM_REGISTER_COMPONENT(
  "buoy_gazebo.components.LatentData",
  LatentData)
}  // namespace components

}  // namespace buoy_gazebo

#endif  // LATENTDATA__LATENTDATA__LATENTDATA_HPP_
