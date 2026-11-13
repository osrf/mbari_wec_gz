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

#ifndef INCWAVESURFACE__INCWAVESURFACEVISUAL_HPP_
#define INCWAVESURFACE__INCWAVESURFACEVISUAL_HPP_

#include <memory>

#include <gz/sim/System.hh>


namespace buoy_gazebo
{
// Forward declaration
class IncWaveSurfaceVisualPrivate;

/// \brief Draws the incident wave surface as an Ogre-next dynamic mesh.
///
/// A visual plugin: put it inside a <visual> of a static model. It requests
/// the wave components from the IncWaveSurface plugin once, then computes the
/// surface on a worker thread and rewrites the mesh's vertex buffer on the
/// render thread. It works for every wave type, since it uses the components
/// FreeSurfaceHydrodynamics generated. It needs the ogre2 render engine.
///
/// The surface is the FreeSurfaceHydrodynamics waves, exact at every vertex,
/// plus optional chop: a periodic FFT patch (Tessendorf) holding only the band
/// above the FreeSurfaceHydrodynamics cutoff, so the two never overlap. The
/// physics doesn't see the chop.
///
/// SDF parameters, all optional:
/// * `<follow_link>`: name of the link the mesh stays centered on, defaults to
///   "Buoy". Empty: centered on the origin.
/// * `<update_rate>`: surface updates per second of wall-clock time, defaults
///   to 30. 0: every rendered frame.
/// * Mesh. Left out, they're chosen from the wave components:
///   * `<size>`: full width of the square mesh (m), about two peak
///     wavelengths
///   * `<fine_size>`: full width of the center, where the spacing is finest
///     (m), defaults to 32
///   * `<fine_spacing>`: vertex spacing in the center (m)
///   * `<coarse_spacing>`: largest vertex spacing, toward the edges (m)
///   * `<max_vertices_per_axis>`: defaults to 255
///   * `<skirt>`: a flat ring this far out (m) carries the surface to the
///     horizon, defaults to 10000. 0: none.
/// * Chop:
///   * `<chop>`: none, tail or wind, defaults to tail. tail continues each
///     direction's spectrum past the cutoff. wind adds a Pierson-Moskowitz
///     wind sea past the cutoff.
///   * `<wind_speed>`: m/s, for wind, defaults to 8
///   * `<wind_direction>`: compass degrees the wind comes from, like WaveDir.
///     Defaults to the mean direction of the waves.
///   * `<chop_tail_exponent>`: p in S ~ omega^-p, for tail, defaults to 5
///   * `<chop_spreading>`: s of the cos^2s(theta/2) spreading, defaults to 2
///   * `<chop_crossover>`: half-width of the cross-fade at the cutoff, as a
///     fraction of it, defaults to 0.1
///   * `<choppiness>`: scale of the horizontal displacement that sharpens
///     crests, defaults to 1
///   * `<chop_patch_size>`: period of the chop patch (m), defaults to 64
///   * `<chop_resolution>`: FFT cells per side, a power of 2, defaults to 256
///   * `<chop_seed>`: defaults to 1
///   * `<chop_normal_map>`: also draw the chop as a normal map, so it shows
///     across the whole surface and not only near the center, where the
///     vertices are close enough for it. Defaults to true.
/// * Look:
///   * `<color>`: diffuse color, defaults to "0.02 0.15 0.25 1"
///   * `<opacity>`: 0 to 1, defaults to 0.75. Below 1 the underwater parts
///     show through; the reflections stay.
///   * `<roughness>`: defaults to 0.3. Lower makes the sun glints so small
///     that they flicker as the chop moves.
///   * `<environment_map>`: cubemap the surface reflects, defaults to the sky
///     gz-rendering draws
/// * `<threads>`: threads to compute with, defaults to half the cores, at
///   most 8
class IncWaveSurfaceVisual : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  IncWaveSurfaceVisual();

  /// \brief Destructor
  ~IncWaveSurfaceVisual() override;

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
  /// \brief Private data pointer
  std::unique_ptr<IncWaveSurfaceVisualPrivate> dataPtr;
};
}  // namespace buoy_gazebo

#endif  // INCWAVESURFACE__INCWAVESURFACEVISUAL_HPP_
