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

#ifndef CONTROLLERS__SERVICESNOTIMPLEMENTED__NOOPCONTROLLER_HPP_
#define CONTROLLERS__SERVICESNOTIMPLEMENTED__NOOPCONTROLLER_HPP_

#include <gz/sim/System.hh>
#include <memory>

namespace buoy_gazebo
{
// Forward declarations.
struct NoOpControllerPrivate;

/// \brief ROS2 NoOp Controller node and accepting all unimplemented commands

/// SDF parameters:
/// * `<namespace>`: Namespace for ROS node, defaults to scoped name
/// * `<node_name>`: ROS2 node name, defaults to "noop_controller"
class NoOpController
  : public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  /// \brief Constructor
  NoOpController();

  /// \brief Destructor
  ~NoOpController() override;

  // Documentation inherited
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<NoOpControllerPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // CONTROLLERS__SERVICESNOTIMPLEMENTED__NOOPCONTROLLER_HPP_
