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

#ifndef CONTROLLERS__XBOWAHRS__XBOWAHRS_HPP_
#define CONTROLLERS__XBOWAHRS__XBOWAHRS_HPP_

#include <ignition/gazebo/System.hh>
#include <memory>

namespace buoy_gazebo
{
// Forward declarations.
struct XBowAHRSPrivate;

/// SDF parameters:
/// * `<namespace>`: Namespace for ROS node, defaults to scoped name
/// * `<node_name>`: ROS2 node name, defaults to "xbow_ahrs"
/// * `<xb_topic>`: ROS2 topic to publish XBRecord, defaults to "xb_record"
/// * `<imu_topic>`: ROS2 topic to publish Imu, defaults to "xb_imu"
/// * `<publish_rate>`: ROS2 topic publish rate, defaults to 10Hz
class XBowAHRS
  : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPostUpdate
{
public:
  /// \brief Constructor
  XBowAHRS();

  /// \brief Destructor
  ~XBowAHRS() override;

  // Documentation inherited
  void Configure(
    const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventMgr) override;

  // Documentation inherited
  void PostUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    const ignition::gazebo::EntityComponentManager & _ecm) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<XBowAHRSPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // CONTROLLERS__XBOWAHRS__XBOWAHRS_HPP_
