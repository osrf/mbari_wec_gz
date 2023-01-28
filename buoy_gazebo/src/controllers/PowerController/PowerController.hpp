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

#ifndef CONTROLLERS__POWERCONTROLLER__POWERCONTROLLER_HPP_
#define CONTROLLERS__POWERCONTROLLER__POWERCONTROLLER_HPP_

#include <memory>

#include <gz/sim/System.hh>

namespace buoy_gazebo
{
// Forward declarations.
struct PowerControllerPrivate;

/// \brief ROS2 Power Controller node for publishing PCRecord and accepting power commands
/// Uses parameter to set publish rate (PCPackRate).
/// Uses ros_gz_bridge and use_sim_time to get /clock from gazebo for command timing.

/// SDF parameters:
/// * `<namespace>`: Namespace for ROS node, defaults to scoped name
/// * `<node_name>`: ROS2 node name, defaults to "power_controller"
/// * `<topic>`: ROS2 topic to publish to, defaults to "pc_record"
/// * `<publish_rate>`: ROS2 topic publish rate, defaults to 10Hz
class PowerController
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  /// \brief Constructor
  PowerController();

  /// \brief Destructor
  ~PowerController() override;

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
  std::unique_ptr<PowerControllerPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // CONTROLLERS__POWERCONTROLLER__POWERCONTROLLER_HPP_
