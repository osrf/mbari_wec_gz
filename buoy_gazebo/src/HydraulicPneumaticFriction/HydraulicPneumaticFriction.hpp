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

#ifndef HYDRAULICPNEUMATICFRICTION__HYDRAULICPNEUMATICFRICTION_HPP_
#define HYDRAULICPNEUMATICFRICTION__HYDRAULICPNEUMATICFRICTION_HPP_

#include <ignition/gazebo/System.hh>

#include <memory>
#include <vector>


namespace buoy_gazebo
{
// Forward declaration
class HydraulicPneumaticFrictionPrivate;
class HydraulicPneumaticFriction : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  /// \brief Constructor
  HydraulicPneumaticFriction();

  /// \brief Destructor
  ~HydraulicPneumaticFriction() override = default;

  // Documentation inherited
  void Configure(
    const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

private:
  /// \brief Private data pointer
  std::unique_ptr<HydraulicPneumaticFrictionPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // HYDRAULICPNEUMATICFRICTION__HYDRAULICPNEUMATICFRICTION_HPP_
