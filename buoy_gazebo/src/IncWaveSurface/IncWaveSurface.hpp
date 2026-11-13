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

/// \brief Draws the incident wave surface around the buoy in the GUI.
///
/// Computes eta on a grid of vertices from the shared FreeSurfaceHydrodynamics
/// object in the IncWaveState component (created by the IncidentWaves plugin)
/// and sends the grid as a triangle-list marker to the GUI's /marker service.
/// The grid is centered on the buoy, snapped to whole cells so the vertices
/// stay fixed in the world as the buoy moves. Heights are computed on a worker
/// thread, and nothing is computed while no GUI is serving /marker.
///
/// SDF parameters:
/// * `<grid_size>`: number of vertices along x and y, defaults to "64 64"
/// * `<grid_spacing>`: distance between vertices along x and y in meters,
///   defaults to "2.0 2.0"
/// * `<publish_rate>`: surface updates per second of wall-clock time,
///   defaults to 10
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
