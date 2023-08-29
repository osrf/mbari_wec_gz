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
    if (!equal) {
      return false;
    }

    // since sizes are equal, iterate over all points (or none)
    std::size_t idx = 0U;
    for (; idx < this->points.size(); ++idx) {
      equal &= (this->points[idx] == that.points[idx]);
    }

    equal &= this->valid == that.valid;

    return equal;
  }
};

struct AirSpring
{
  bool valid{false};
  double force{0.0};
  double T{0.0};
  double dQ_dt{0.0};
  double piston_position{0.0};
  double piston_velocity{0.0};

  bool operator==(const AirSpring & that) const
  {
    bool equal = (this->valid == that.valid);
    equal &= (this->force == that.force);
    equal &= (this->T == that.T);
    equal &= (this->dQ_dt == that.dQ_dt);

    return equal;
  }
};

struct ElectroHydraulic
{
  bool valid{false};
  double inst_power{0.0};
  double rpm{0.0};
  double motor_drive_i2r_loss{0.0};
  double motor_drive_friction_loss{0.0};
  double motor_drive_switching_loss{0.0};
  double battery_i2r_loss{0.0};
};

/// \brief latent data that is modeled but not directly observed for LatentData message in ROS 2
struct LatentData
{
  IncWaveHeights inc_wave_heights;
  AirSpring upper_spring;
  AirSpring lower_spring;
  ElectroHydraulic electro_hydraulic;

  bool valid() const
  {
    return inc_wave_heights.valid && \
           upper_spring.valid && lower_spring.valid && \
           electro_hydraulic.valid;
  }

  bool operator==(const LatentData & that) const
  {
    bool equal = (this->inc_wave_heights == that.inc_wave_heights);
    equal &= (this->upper_spring == that.upper_spring);
    equal &= (this->lower_spring == that.lower_spring);
    // equal &= fabs(this-> - that.) < 1e-7F;
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
