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

#ifndef INCWAVESURFACE__WAVESURFACEFIELD_HPP_
#define INCWAVESURFACE__WAVESURFACEFIELD_HPP_

#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "IncWaveComponents.hpp"


namespace buoy_gazebo
{
namespace inc_wave_surface
{
/// \brief Where the short waves above the FreeSurfaceHydrodynamics cutoff
/// come from
enum class ChopMode
{
  /// \brief No chop
  kNone,
  /// \brief Continue each direction's spectrum past the cutoff
  kTail,
  /// \brief Wind sea (Pierson-Moskowitz) past the cutoff
  kWind
};

/// \brief Settings for WaveSurfaceField
struct SurfaceParams
{
  // Mesh. Values <= 0 are chosen from the wave components.
  /// \brief Full width of the square mesh (m)
  double size{0.0};
  /// \brief Full width of the center, where the spacing is finest (m)
  double fineSize{32.0};
  /// \brief Vertex spacing in the center (m)
  double fineSpacing{0.0};
  /// \brief Largest vertex spacing, toward the edges (m)
  double coarseSpacing{0.0};
  /// \brief Vertex budget along each axis
  int maxVerticesPerAxis{255};
  /// \brief A flat ring this far out (m) carries the surface to the horizon.
  /// 0: none
  double skirt{10000.0};

  // Chop
  ChopMode chop{ChopMode::kTail};
  /// \brief The chop repeats with this period (m)
  double chopPatchSize{64.0};
  /// \brief FFT cells along each side, rounded up to a power of 2
  int chopResolution{256};
  /// \brief p in the tail S(omega) ~ omega^-p, for ChopMode::kTail
  double chopTailExponent{5.0};
  /// \brief Wind speed (m/s), for ChopMode::kWind
  double windSpeed{8.0};
  /// \brief Direction the wind blows to, rad CCW from +x. NaN: the mean
  /// direction of the waves
  double windDirection{std::numeric_limits<double>::quiet_NaN()};
  /// \brief s of the cos^2s(theta/2) directional spreading of the chop
  double chopSpreading{2.0};
  /// \brief Half-width of the cross-fade at the cutoff, fraction of it
  double chopCrossover{0.1};
  /// \brief Scale of the horizontal displacement that sharpens crests
  double choppiness{1.0};
  unsigned int chopSeed{1};
  /// \brief Also make a normal map of the chop, so it shows everywhere and
  /// not only where the vertices are close enough for it. The vertex normals
  /// then leave the chop out.
  bool chopNormalMap{true};

  /// \brief Threads to compute with. 0: pick from the hardware
  int threads{0};
};

// Forward declaration
class WorkerPool;

/// \brief Computes the wave surface on a mesh centered on a followed point.
///
/// The FreeSurfaceHydrodynamics part is exact at every vertex. Each direction
/// entry depends only on xi = x cos(beta) + y sin(beta), so it is evaluated
/// on a fine line of xi and interpolated (cubic Hermite, with exact slopes).
/// The chop is a periodic FFT patch (Tessendorf) that holds only the band
/// above the FreeSurfaceHydrodynamics cutoff. It fades out where the mesh is
/// too coarse to show it.
///
/// The mesh is a square grid, finest in the center and coarser toward the
/// edges, with the same spacing along x and y, and an optional flat ring far
/// out. The chop normal map tiles the surface once per chop patch, through
/// the vertex UVs.
class WaveSurfaceField
{
public:
  /// \brief Floats per vertex: position xyz, normal xyz, tangent xyz, uv
  static constexpr int kFloatsPerVertex = 11;

  WaveSurfaceField();
  ~WaveSurfaceField();

  /// \brief Set up the mesh, the lines and the chop
  void Init(const SurfaceParams & _params, const IncWaveComponents & _waves);

  /// \brief Keep the mesh centered near a point. The center moves only when
  /// the point leaves the middle of the fine center, and snaps to the fine
  /// spacing.
  void Follow(double _x, double _y);

  /// \brief Compute the vertices at sim time _t (s). Positions are relative
  /// to the center.
  void Compute(double _t, std::vector<float> & _vertices);

  /// \brief Vertices along each side
  int VerticesPerSide() const;

  /// \brief Triangle list, counter-clockwise seen from above
  const std::vector<std::uint32_t> & Indices() const;

  double CenterX() const;
  double CenterY() const;

  /// \brief Half-width of a box around the center that holds every vertex
  double BoundHalfWidth() const;

  /// \brief Half-height of a box around the center that holds every vertex
  double BoundHalfHeight() const;

  /// \brief What Init chose, for the log
  const std::string & Summary() const;

  /// \brief Texels along each side of the chop normal map. 0: no normal map.
  int NormalMapSize() const;

  /// \brief The chop normal map from the last Compute, one entry per mip
  /// level from the full size down to 1 x 1. Two signed bytes per texel:
  /// the x and y of the tangent-space normal, scaled by 127.
  const std::vector<std::vector<std::int8_t>> & NormalMapLevels() const;

  /// \brief Wave height and slopes from the FreeSurfaceHydrodynamics
  /// components alone, at a point the last Compute covered. For testing.
  double WavesAt(double _x, double _y, double & _slopeX, double & _slopeY) const;

private:
  /// \brief eta and d eta / d xi of one direction entry along a line of xi
  struct Line
  {
    double cosBeta{1.0};
    double sinBeta{0.0};
    /// \brief Sample spacing
    double dxi{1.0};
    /// \brief Half the span of xi the mesh covers
    double halfSpan{0.0};
    /// \brief xi of the first sample, for the last Compute
    double xi0{0.0};
    std::size_t numSamples{0};
    std::vector<double> A;
    std::vector<double> omega;
    std::vector<double> k;
    std::vector<double> phase;
    /// \brief exp(i k dxi), to step along the line
    std::vector<double> stepRe;
    std::vector<double> stepIm;
    std::vector<double> value;
    std::vector<double> slope;
  };

  void InitChop(const IncWaveComponents & _waves, double _omegaCut);
  void ComputeLines(double _t);
  void ComputeChop(double _t);
  void InverseFft(std::complex<float> * _data) const;
  void InverseFft2d(std::vector<std::complex<float>> & _data);
  void SampleChop(double _x, double _y, float _out[5]) const;
  void ComputeNormalMap();

  SurfaceParams params;
  int numThreads{1};
  std::unique_ptr<WorkerPool> pool;
  std::vector<Line> lines;

  // Mesh, the same along x and y
  std::vector<double> offsets;
  std::vector<float> chopWeight;
  /// \brief Tapers the waves to flat at the edge of the wave part, so it
  /// meets the flat skirt smoothly
  std::vector<double> edgeWeight;
  std::vector<std::uint32_t> indices;
  double fineSpacing{1.0};
  double fineHalf{16.0};
  /// \brief Half-width of the wave part of the mesh; the skirt is beyond
  double meshHalf{1.0};
  bool haveCenter{false};
  double centerX{0.0};
  double centerY{0.0};
  double boundHalfWidth{1.0};
  double boundHalfHeight{1.0};

  // Chop
  bool chopOn{false};
  int chopN{0};
  double chopCell{1.0};
  /// \brief Standard deviation of the chop height (m)
  double chopSigma{0.0};
  std::vector<std::complex<double>> chopAmplitude;
  std::vector<double> chopOmega;
  std::vector<std::complex<float>> chopSpectrum[3];
  /// \brief eta, d eta/dx, d eta/dy, dx, dy at each cell
  std::vector<float> chopTile;
  std::vector<int> fftReverse;
  std::vector<std::complex<float>> fftTwiddle;
  bool normalMapOn{false};
  std::vector<std::vector<std::int8_t>> normalLevels;

  std::string summary;
};
}  // namespace inc_wave_surface
}  // namespace buoy_gazebo

#endif  // INCWAVESURFACE__WAVESURFACEFIELD_HPP_
