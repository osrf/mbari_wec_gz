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

#ifndef INCIDENTWAVES__INCIDENTWAVES_HPP_
#define INCIDENTWAVES__INCIDENTWAVES_HPP_

#include <memory>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/transport.hh>


namespace buoy_gazebo
{
// Forward declaration
class IncidentWavesPrivate;
class IncidentWaves : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  IncidentWaves();

  /// \brief Destructor
  ~IncidentWaves() override = default;

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

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher pistonvel_pub;

  /// \brief Private data pointer
  std::unique_ptr<IncidentWavesPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // INCIDENTWAVES__INCIDENTWAVES_HPP_
