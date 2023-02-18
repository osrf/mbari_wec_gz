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

#ifndef CONTROLLERS__TREFOILCONTROLLER__TREFOILCONTROLLER_HPP_
#define CONTROLLERS__TREFOILCONTROLLER__TREFOILCONTROLLER_HPP_

#include <memory>

#include <gz/sim/System.hh>

namespace buoy_gazebo
{
// Forward declarations.
struct TrefoilControllerPrivate;

/// SDF parameters:
/// * `<namespace>`: Namespace for ROS node, defaults to scoped name
/// * `<node_name>`: ROS2 node name, defaults to "xbow_ahrs"
/// * `<tf_topic>`: ROS2 topic to publish TFRecord, defaults to "tf_record"
/// * `<imu_topic>`: ROS2 topic to publish Imu, defaults to "tf_imu"
/// * `<publish_rate>`: ROS2 topic publish rate, defaults to 10Hz
class TrefoilController
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  /// \brief Constructor
  TrefoilController();

  /// \brief Destructor
  ~TrefoilController() override;

  // Documentation inherited
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  // Documentation inherited
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<TrefoilControllerPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // CONTROLLERS__TREFOILCONTROLLER__TREFOILCONTROLLER_HPP_
