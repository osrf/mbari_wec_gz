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

#ifndef LATENT_DATA__INCWAVEHEIGHT__INCWAVEHEIGHT_HPP_
#define LATENT_DATA__INCWAVEHEIGHT__INCWAVEHEIGHT_HPP_

#include <memory>

#include <gz/sim/System.hh>

namespace buoy_gazebo
{
// Forward declarations.
struct IncWaveHeightPrivate;

/// \brief ROS 2 Incident Wave Height node for requesting wave heights at given positions
/// Currently accepts incident wave height requests.
/// Uses ros_gz_bridge

/// SDF parameters:
/// * `<namespace>`: Namespace for ROS node, defaults to scoped name
/// * `<node_name>`: ROS2 node name, defaults to "inc_wave_height"
/// * `<service>`: ROS2 service name, defaults to "inc_wave_height"
class IncWaveHeight
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  /// \brief Constructor
  IncWaveHeight();

  /// \brief Destructor
  ~IncWaveHeight() override;

  // Documentation inherited
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  // Documentation inherited
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<IncWaveHeightPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // LATENT_DATA__INCWAVEHEIGHT__INCWAVEHEIGHT_HPP_
