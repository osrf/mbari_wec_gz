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

#ifndef RENDERING__TESSENDORFWAVES__TESSENDORFWAVES_HPP_
#define RENDERING__TESSENDORFWAVES__TESSENDORFWAVES_HPP_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace buoy
{
#pragma pack(push,1)
struct Vertices {
  float px, py, pz;
  float nx, ny, nz;
  float texx, texy;
};
#pragma pack(pop)

class TessendorfWavesPrivate;

class TessendorfWaves
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
  /// \brief Constructor
  TessendorfWaves();
  
  /// \brief Destructor
  ~TessendorfWaves() override = default;
  
  /// Documentation inherited
  void Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr) final;
  
  /// Documentation inherited
  void PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) final;

private:
  /// \brief Private data pointer
  std::unique_ptr<TessendorfWavesPrivate> dataPtr;
};

}  // namespace buoy

std::ostream & operator<<(std::ostream &os, const buoy::Vertices &verts);

#endif  // RENDERING__TESSENDORFWAVES__TESSENDORFWAVES_HPP_

