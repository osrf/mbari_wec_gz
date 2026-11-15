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
/// come from. Either way they're only drawn; the physics doesn't see them.
enum class ChopMode
{
  /// \brief No chop
  kNone,
  /// \brief Continue each direction's spectrum past the cutoff: its level
  /// just below the cutoff, falling as omega^-p, in its direction. The chop
  /// follows the sea. Nothing for spectra that stop short of the cutoff, like
  /// a single wave.
  kTail,
  /// \brief The part past the cutoff of a Pierson-Moskowitz wind sea for
  /// windSpeed, toward windDirection, whatever the waves below the cutoff
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
/// The FreeSurfaceHydrodynamics waves are the sum LinearIncidentWave::eta
/// computes, over the components the server realized:
///
///   eta(x, y, t) = sum over the directions d, and the components c of d, of
///                  A_c cos(k_c xi_d - omega_c t + phase_c)
///   where xi_d = x cos(beta_d) + y sin(beta_d)
///
/// Both use the fact that the components of direction d depend on x and y
/// only through xi_d, the distance along d. Each direction is a 1D profile,
/// the same along every crest, and the surface is the sum of those profiles.
/// They differ in where they evaluate the profiles.
///
/// LinearIncidentWave::eta evaluates them at one point. It projects the point
/// onto each direction (its xx is xi_d) and sums a cosine per component, plus
/// a sine per component when asked for the slopes. That's exact, and right
/// for the physics, which needs one point per step. For a mesh it would be a
/// cosine per component per vertex every update: 9,500 components on 66,000
/// vertices for the default sea, which takes seconds.
///
/// Here, every vertex on the same crest of direction d has the same xi_d, so
/// each direction's profile is computed once and shared. Each update:
///
///   1. Sums each direction's components along a line of xi_d that spans the
///      mesh, at 12 samples per shortest wavelength, and keeps the height and
///      the slope d eta / d xi_d at each sample (ComputeLines).
///   2. At each vertex, finds its xi_d on each line, interpolates between
///      the two samples around it (cubic Hermite, from the height and slope
///      at both), and adds up the directions. The slopes along x and y are
///      the slope along d times cos(beta_d) and sin(beta_d), as in eta
///      (FilteredWavesAt).
///
/// Compared with eta: step 1 steps each component along its line by complex
/// multiplies, with no cosines in the loop, and step 2 costs a few lookups
/// per direction per vertex, however many components there are. The heights
/// are interpolated rather than exact, and match eta to a fraction of a
/// millimeter. Unlike eta, this doesn't compute the surface velocities, and
/// it drops the short waves at coarse vertices, as below.
///
/// Each vertex holds the FreeSurfaceHydrodynamics waves its spacing can show:
/// all of them where the mesh is fine, and only the longer ones farther out.
/// Waves shorter than a few spacings would alias there, giving each vertex a
/// random slope, so patches of the mesh would flash light and dark as the
/// waves pass. Each line keeps running sums over bands of k, from the longest
/// waves up, so a vertex reads the sum up to its own cutoff.
///
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
  /// the x and y of the tangent-space normal, scaled by 127. Each level is
  /// computed from the spectrum and holds only the waves it can show.
  const std::vector<std::vector<std::int8_t>> & NormalMapLevels() const;

  /// \brief Wave height and slopes from all of the FreeSurfaceHydrodynamics
  /// components, at a point the last Compute covered. For testing.
  double WavesAt(double _x, double _y, double & _slopeX, double & _slopeY) const;

private:
  /// \brief One direction entry as a 1D profile: eta and d eta / d xi at
  /// samples of xi = x cos(beta) + y sin(beta), dxi apart
  struct Line
  {
    double cosBeta{1.0};
    double sinBeta{0.0};
    /// \brief log |cos(beta)| and log |sin(beta)|, for the band limit
    double logAbsCos{0.0};
    double logAbsSin{0.0};
    /// \brief Sample spacing
    double dxi{1.0};
    /// \brief Half the span of xi the mesh covers
    double halfSpan{0.0};
    /// \brief xi of the first sample, for the last Compute
    double xi0{0.0};
    std::size_t numSamples{0};
    /// \brief The components, sorted by k
    std::vector<double> A;
    std::vector<double> omega;
    std::vector<double> k;
    std::vector<double> phase;
    /// \brief Band b holds the components from bandStart[b] up to
    /// bandStart[b + 1]
    std::vector<std::size_t> bandStart;
    /// \brief exp(i k dxi), to step along the line
    std::vector<double> stepRe;
    std::vector<double> stepIm;
    /// \brief At [sample * numBands + b]: the sum over bands 0 to b
    std::vector<double> value;
    std::vector<double> slope;
  };

  void InitChop(const IncWaveComponents & _waves, double _omegaCut);
  void ComputeLines(double _t);
  /// \brief Like WavesAt, but only with the waves a vertex can show where
  /// the log of the spacing is _logSpacingX along x and _logSpacingY along
  /// y. -infinity: all of them.
  double FilteredWavesAt(
    double _x, double _y, double _logSpacingX, double _logSpacingY,
    double & _slopeX, double & _slopeY) const;
  void ComputeChop(double _t);
  void InverseFft(std::complex<float> * _data, int _n) const;
  void InverseFft2d(std::vector<std::complex<float>> & _data, int _n);
  void SampleChop(double _x, double _y, float _out[5]) const;
  void ComputeNormalMap();

  SurfaceParams params;
  int numThreads{1};
  std::unique_ptr<WorkerPool> pool;
  std::vector<Line> lines;
  /// \brief Bands of k the lines are split into. Band 0 holds every wave the
  /// coarsest vertices can show.
  int numBands{1};

  // Mesh, the same along x and y
  std::vector<double> offsets;
  /// \brief log of the spacing at each vertex, for the band limit
  std::vector<double> logSpacing;
  /// \brief log of the largest spacing of the wave part
  double logMaxSpacing{0.0};
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
  /// \brief h(k, t) of each mode, for the last Compute
  std::vector<std::complex<float>> chopH;
  /// \brief eta, d eta/dx, d eta/dy, dx, dy at each cell
  std::vector<float> chopTile;
  /// \brief exp(2 pi i j / chopN), for every FFT size up to chopN
  std::vector<std::complex<float>> fftTwiddle;
  bool normalMapOn{false};
  std::vector<std::vector<std::int8_t>> normalLevels;

  std::string summary;
};
}  // namespace inc_wave_surface
}  // namespace buoy_gazebo

#endif  // INCWAVESURFACE__WAVESURFACEFIELD_HPP_
