// Copyright 2026 Monterey Bay Aquarium Research Institute
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

#ifndef INCWAVESURFACE__INCWAVESURFACE_HPP_
#define INCWAVESURFACE__INCWAVESURFACE_HPP_

#include <memory>

#include <gz/sim/System.hh>


namespace buoy_gazebo
{
// Forward declaration
class IncWaveSurfacePrivate;

/// \brief Serves the incident wave field to the GUI, where the
/// IncWaveSurfaceVisual plugin draws it.
///
/// Reads the shared FreeSurfaceHydrodynamics object in the IncWaveState
/// component (created by the IncidentWaves plugin) once and serves its wave
/// components on /world/<world>/inc_wave_components (reply
/// gz::msgs::Double_V, layout in IncWaveComponents.hpp). It sends the
/// components FreeSurfaceHydrodynamics generated rather than the spectrum
/// settings, so it works the same for every wave type.
///
/// No SDF parameters.
class IncWaveSurface : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  /// \brief Constructor
  IncWaveSurface();

  /// \brief Destructor
  ~IncWaveSurface() override;

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
  /// \brief Private data pointer
  std::unique_ptr<IncWaveSurfacePrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // INCWAVESURFACE__INCWAVESURFACE_HPP_
