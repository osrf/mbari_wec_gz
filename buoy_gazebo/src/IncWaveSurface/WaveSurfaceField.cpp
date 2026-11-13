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

#include "WaveSurfaceField.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>


namespace buoy_gazebo
{
namespace inc_wave_surface
{
namespace
{
/// \brief Gravity (m/s^2), the value LinearIncidentWave uses
constexpr double kGravity = 9.81;

constexpr double kPi = 3.14159265358979323846;

double Clamp(double _value, double _low, double _high)
{
  return std::min(std::max(_value, _low), _high);
}

/// \brief Normalizing factor of cos^2s(theta/2) over a full circle
double SpreadingNorm(double _s)
{
  return std::exp(std::lgamma(_s + 1.0) - std::lgamma(_s + 0.5)) / (2.0 * std::sqrt(kPi));
}

/// \brief cos^2s(theta/2) spreading, _dtheta away from the mean direction
double Spreading(double _dtheta, double _s, double _norm)
{
  const double c = std::cos(0.5 * std::remainder(_dtheta, 2.0 * kPi));
  return _norm * std::pow(std::abs(c), 2.0 * _s);
}

/// \brief Vertex offsets from the center out to _half along one half axis:
/// _fine out to _fineHalf, then growing by _growth per cell up to _coarse
std::vector<double> HalfAxis(
  double _fine, double _fineHalf, double _coarse, double _half, double _growth)
{
  std::vector<double> offsets{0.0};
  double s = 0.0;
  double h = _fine;
  while (s < _half - 1e-9) {
    if (s >= _fineHalf - 1e-9) {
      h = std::min(h * _growth, _coarse);
    }
    s += h;
    offsets.push_back(s);
  }
  return offsets;
}

int NextPowerOfTwo(int _n)
{
  int p = 1;
  while (p < _n) {
    p <<= 1;
  }
  return p;
}
}  // namespace

/// \brief Runs tasks on a fixed set of threads. Run is called from one
/// thread at a time.
class WorkerPool
{
public:
  /// \param[in] _threads Threads in total, counting the one calling Run
  explicit WorkerPool(int _threads)
  {
    for (int i = 1; i < _threads; ++i) {
      this->threads.emplace_back(&WorkerPool::Loop, this);
    }
  }

  ~WorkerPool()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->stop = true;
    }
    this->startCv.notify_all();
    for (auto & thread : this->threads) {
      thread.join();
    }
  }

  /// \brief Run _fn(i) for each i in [0, _n), returning when all are done
  void Run(int _n, const std::function<void(int)> & _fn)
  {
    if (_n <= 0) {
      return;
    }
    if (this->threads.empty() || _n == 1) {
      for (int i = 0; i < _n; ++i) {
        _fn(i);
      }
      return;
    }
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->task = &_fn;
      this->count = _n;
      this->next = 0;
      this->active = static_cast<int>(this->threads.size());
      ++this->generation;
    }
    this->startCv.notify_all();
    for (int i = this->next++; i < _n; i = this->next++) {
      _fn(i);
    }
    std::unique_lock<std::mutex> lock(this->mutex);
    this->doneCv.wait(lock, [this] {return this->active == 0;});
    this->task = nullptr;
  }

private:
  void Loop()
  {
    std::uint64_t seen = 0;
    while (true) {
      const std::function<void(int)> * fn = nullptr;
      int n = 0;
      {
        std::unique_lock<std::mutex> lock(this->mutex);
        this->startCv.wait(
          lock, [this, seen] {return this->stop || this->generation != seen;});
        if (this->stop) {
          return;
        }
        seen = this->generation;
        fn = this->task;
        n = this->count;
      }
      for (int i = this->next++; i < n; i = this->next++) {
        (*fn)(i);
      }
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (--this->active == 0) {
          this->doneCv.notify_one();
        }
      }
    }
  }

  std::vector<std::thread> threads;
  std::mutex mutex;
  std::condition_variable startCv;
  std::condition_variable doneCv;
  const std::function<void(int)> * task{nullptr};
  int count{0};
  std::atomic<int> next{0};
  int active{0};
  std::uint64_t generation{0};
  bool stop{false};
};

//////////////////////////////////////////////////
WaveSurfaceField::WaveSurfaceField() = default;

//////////////////////////////////////////////////
WaveSurfaceField::~WaveSurfaceField() = default;

//////////////////////////////////////////////////
void WaveSurfaceField::Init(const SurfaceParams & _params, const IncWaveComponents & _waves)
{
  this->params = _params;
  this->numThreads = _params.threads;
  if (this->numThreads <= 0) {
    const int hardware = static_cast<int>(std::thread::hardware_concurrency());
    this->numThreads = std::max(1, std::min(8, hardware / 2));
  }
  this->pool = std::make_unique<WorkerPool>(this->numThreads);

  // Scales of the waves
  double aMax = 0.0;
  double kPeak = 0.0;
  double sumA = 0.0;
  double omegaMax = 0.0;
  std::size_t numComponents = 0;
  for (const auto & entry : _waves.entries) {
    for (std::size_t i = 0; i < entry.A.size(); ++i) {
      const double a = std::abs(entry.A[i]);
      sumA += a;
      if (a > aMax) {
        aMax = a;
        kPeak = entry.k[i];
      }
      omegaMax = std::max(omegaMax, entry.omega[i]);
    }
    numComponents += entry.A.size();
  }
  double kShort = 0.0;
  for (const auto & entry : _waves.entries) {
    for (std::size_t i = 0; i < entry.A.size(); ++i) {
      if (std::abs(entry.A[i]) >= 0.01 * aMax) {
        kShort = std::max(kShort, entry.k[i]);
      }
    }
  }
  const double lambdaPeak = kPeak > 0.0 ? 2.0 * kPi / kPeak : 100.0;
  const double lambdaShort = kShort > 0.0 ? 2.0 * kPi / kShort : lambdaPeak;
  double omegaCut = 2.0 * kPi * _waves.maxFreqHz;
  if (omegaCut <= 0.0) {
    omegaCut = omegaMax > 0.0 ? omegaMax : kPi;
  }

  const bool wantChop =
    _params.chop != ChopMode::kNone && _params.chopPatchSize > 0.0;
  this->chopN = NextPowerOfTwo(std::min(std::max(_params.chopResolution, 16), 1024));
  this->chopCell = std::max(_params.chopPatchSize, 1e-3) / this->chopN;

  // Mesh: at least two peak wavelengths, and 512 m unless that's more than 32
  // of them. Fine enough near the center for the shortest waves (or the
  // chop), and about 16 vertices per peak wavelength at the edges.
  const double autoSize = std::max(2.0 * lambdaPeak, std::min(512.0, 32.0 * lambdaPeak));
  const double halfSize = 0.5 *
    (_params.size > 0.0 ? _params.size : Clamp(autoSize, 128.0, 4000.0));
  double coarse = _params.coarseSpacing > 0.0 ?
    _params.coarseSpacing : Clamp(lambdaPeak / 16.0, 1.0, 32.0);
  double fine = _params.fineSpacing > 0.0 ?
    _params.fineSpacing : (wantChop ? this->chopCell : Clamp(lambdaShort / 8.0, 0.1, 2.0));
  fine = std::min(fine, coarse);
  const double fineHalfSize = Clamp(0.5 * _params.fineSize, fine, halfSize);

  // Fit the vertex budget: grow the cells faster, then coarsen
  const int maxHalf = std::max(2, (_params.maxVerticesPerAxis - 1) / 2);
  double growth = 1.05;
  std::vector<double> half;
  for (int attempt = 0; attempt < 1000; ++attempt) {
    half = HalfAxis(fine, fineHalfSize, coarse, halfSize, growth);
    if (static_cast<int>(half.size()) - 1 <= maxHalf) {
      break;
    }
    if ((halfSize - fineHalfSize) / coarse > 0.5 * maxHalf) {
      coarse *= 1.1;
    } else if (fineHalfSize / fine > 0.75 * maxHalf) {
      fine = std::min(1.1 * fine, coarse);
    } else {
      growth = 1.0 + (growth - 1.0) * 1.1;
    }
  }
  this->fineSpacing = fine;
  this->fineHalf = fineHalfSize;
  this->haveCenter = false;
  this->centerX = 0.0;
  this->centerY = 0.0;

  this->meshHalf = half.back();
  const bool skirt = _params.skirt > this->meshHalf;
  this->offsets.clear();
  if (skirt) {
    this->offsets.push_back(-_params.skirt);
  }
  for (std::size_t i = half.size() - 1; i > 0; --i) {
    this->offsets.push_back(-half[i]);
  }
  this->offsets.insert(this->offsets.end(), half.begin(), half.end());
  if (skirt) {
    this->offsets.push_back(_params.skirt);
  }
  const int side = static_cast<int>(this->offsets.size());

  // Two triangles per cell, counter-clockwise seen from above
  this->indices.clear();
  this->indices.reserve(6 * (side - 1) * (side - 1));
  for (int row = 0; row < side - 1; ++row) {
    for (int col = 0; col < side - 1; ++col) {
      const std::uint32_t a = row * side + col;
      const std::uint32_t b = a + 1;
      const std::uint32_t c = a + side + 1;
      const std::uint32_t d = a + side;
      this->indices.insert(this->indices.end(), {a, b, c, a, c, d});
    }
  }

  // With a skirt, taper the waves to flat over the outer 15% of the wave
  // part. Long triangles out to the skirt would otherwise smear the edge's
  // normals into streaks.
  this->edgeWeight.assign(side, 1.0);
  if (skirt) {
    for (int i = 0; i < side; ++i) {
      const double x = Clamp(
        (this->meshHalf - std::abs(this->offsets[i])) / (0.15 * this->meshHalf), 0.0, 1.0);
      this->edgeWeight[i] = x * x * (3.0 - 2.0 * x);
    }
  }

  this->InitChop(_waves, omegaCut);
  this->normalMapOn = this->chopOn && _params.chopNormalMap;
  this->normalLevels.clear();

  // Fade the chop out where the vertices are too far apart to show it: full
  // up to a third of a quarter of the longest chop wavelength, none from a
  // quarter of it
  this->chopWeight.assign(side, 0.0f);
  if (this->chopOn) {
    const double lambdaCut = 2.0 * kPi * kGravity / (omegaCut * omegaCut);
    const double hZero = std::max(0.25 * lambdaCut, 2.0 * this->chopCell);
    const double hFull = std::min(std::max(1.01 * this->chopCell, hZero / 3.0), 0.9 * hZero);
    for (int i = 0; i < side; ++i) {
      double h;
      if (i == 0) {
        h = this->offsets[1] - this->offsets[0];
      } else if (i == side - 1) {
        h = this->offsets[i] - this->offsets[i - 1];
      } else {
        h = 0.5 * (this->offsets[i + 1] - this->offsets[i - 1]);
      }
      this->chopWeight[i] =
        static_cast<float>(Clamp((hZero - h) / (hZero - hFull), 0.0, 1.0));
    }
  }

  // One line per direction entry, sampled finely enough for cubic Hermite
  // interpolation of its shortest wave
  this->lines.clear();
  for (const auto & entry : _waves.entries) {
    if (entry.A.empty()) {
      continue;
    }
    Line line;
    line.cosBeta = std::cos(entry.beta);
    line.sinBeta = std::sin(entry.beta);
    double kMax = 0.0;
    for (const double k : entry.k) {
      kMax = std::max(kMax, std::abs(k));
    }
    line.dxi = kMax > 0.0 ? Clamp(2.0 * kPi / kMax / 12.0, 0.01, 4.0) : 4.0;
    line.halfSpan =
      this->meshHalf * (std::abs(line.cosBeta) + std::abs(line.sinBeta)) + line.dxi;
    line.numSamples =
      static_cast<std::size_t>(std::ceil(2.0 * line.halfSpan / line.dxi)) + 2;
    line.A = entry.A;
    line.omega = entry.omega;
    line.k = entry.k;
    line.phase = entry.phase;
    line.stepRe.resize(line.k.size());
    line.stepIm.resize(line.k.size());
    for (std::size_t i = 0; i < line.k.size(); ++i) {
      line.stepRe[i] = std::cos(line.k[i] * line.dxi);
      line.stepIm[i] = std::sin(line.k[i] * line.dxi);
    }
    line.value.assign(line.numSamples, 0.0);
    line.slope.assign(line.numSamples, 0.0);
    this->lines.push_back(std::move(line));
  }

  const double chopReach = this->chopOn ? 6.0 * this->chopSigma : 0.0;
  this->boundHalfWidth =
    this->offsets.back() + std::abs(this->params.choppiness) * chopReach + 1.0;
  this->boundHalfHeight = sumA + chopReach + 1.0;

  std::ostringstream summaryStream;
  summaryStream << _waves.entries.size() << " direction(s), " << numComponents
                << " wave components; mesh " << side << " x " << side << " vertices, "
                << 2.0 * this->meshHalf << " m wide, spacing " << fine
                << " m over the center " << 2.0 * fineHalfSize << " m, up to " << coarse
                << " m";
  if (skirt) {
    summaryStream << ", flat out to " << _params.skirt << " m";
  }
  summaryStream << "; chop: ";
  if (!this->chopOn) {
    summaryStream << "none";
  } else {
    if (this->params.chop == ChopMode::kWind) {
      summaryStream << "wind " << this->params.windSpeed << " m/s";
    } else {
      summaryStream << "spectrum tail";
    }
    summaryStream << ", Hs " << 4.0 * this->chopSigma << " m, " << this->params.chopPatchSize
                  << " m patch, " << this->chopCell << " m cells"
                  << (this->normalMapOn ? ", normal map" : "");
  }
  summaryStream << "; " << this->numThreads << " thread(s)";
  this->summary = summaryStream.str();
}

//////////////////////////////////////////////////
void WaveSurfaceField::InitChop(const IncWaveComponents & _waves, double _omegaCut)
{
  this->chopOn = false;
  this->chopSigma = 0.0;
  if (this->params.chop == ChopMode::kNone || this->params.chopPatchSize <= 0.0) {
    return;
  }

  const int n = this->chopN;
  const double dk = 2.0 * kPi / this->params.chopPatchSize;
  const double s = std::max(0.0, this->params.chopSpreading);
  const double norm = SpreadingNorm(s);
  const double p = this->params.chopTailExponent;

  // Mean direction of the waves, weighted by energy
  double meanX = 0.0;
  double meanY = 0.0;
  for (const auto & entry : _waves.entries) {
    double energy = 0.0;
    for (const double a : entry.A) {
      energy += a * a;
    }
    meanX += energy * std::cos(entry.beta);
    meanY += energy * std::sin(entry.beta);
  }
  const double meanDirection = (meanX == 0.0 && meanY == 0.0) ? 0.0 : std::atan2(meanY, meanX);
  const double windDirection =
    std::isnan(this->params.windDirection) ? meanDirection : this->params.windDirection;

  // For the tail: each direction's spectral level just below the cutoff,
  // from its components. A^2 / (2 d omega) is S(omega) for every spectrum
  // type, so this works whether it was given as Hs and Tp or as a table.
  struct Tail
  {
    double beta;
    double level;  // S at the cutoff (m^2 s/rad)
  };
  std::vector<Tail> tails;
  if (this->params.chop == ChopMode::kTail) {
    for (const auto & entry : _waves.entries) {
      const std::size_t num = entry.omega.size();
      if (num < 16) {
        continue;
      }
      std::vector<std::size_t> order(num);
      std::iota(order.begin(), order.end(), 0);
      std::sort(
        order.begin(), order.end(),
        [&entry](std::size_t _a, std::size_t _b) {return entry.omega[_a] < entry.omega[_b];});
      const double top = entry.omega[order.back()];
      // This direction's spectrum stops short of the cutoff: nothing to extend
      if (top < 0.9 * _omegaCut) {
        continue;
      }
      std::size_t first = num;
      while (first > 0 && entry.omega[order[first - 1]] >= 0.8 * top) {
        --first;
      }
      const std::size_t used = num - first;
      if (used < 4) {
        continue;
      }
      const double dOmega = (top - entry.omega[order[first]]) / static_cast<double>(used - 1);
      if (dOmega <= 0.0) {
        continue;
      }
      double level = 0.0;
      for (std::size_t i = first; i < num; ++i) {
        const double a = entry.A[order[i]];
        const double omega = entry.omega[order[i]];
        level += a * a / (2.0 * dOmega) * std::pow(omega / _omegaCut, p);
      }
      tails.push_back({entry.beta, level / static_cast<double>(used)});
    }
  }

  // Directional frequency spectrum of the chop, S(omega, theta)
  auto chopSpectrumAt = [&](double _omega, double _theta) -> double
    {
      if (this->params.chop == ChopMode::kWind) {
        const double u = this->params.windSpeed;
        if (u <= 0.0) {
          return 0.0;
        }
        // Pierson-Moskowitz
        const double r = kGravity / (u * _omega);
        return 0.0081 * kGravity * kGravity / std::pow(_omega, 5.0) *
               std::exp(-0.74 * r * r * r * r) *
               Spreading(_theta - windDirection, s, norm);
      }
      double total = 0.0;
      for (const auto & tail : tails) {
        total += tail.level * std::pow(_omegaCut / _omega, p) *
          Spreading(_theta - tail.beta, s, norm);
      }
      return total;
    };

  // Cross-fade in around the cutoff, so there is neither a gap nor double
  // counted energy where the FreeSurfaceHydrodynamics waves stop
  const double fadeHalfWidth = std::max(0.0, this->params.chopCrossover) * _omegaCut;
  auto crossover = [&](double _omega) -> double
    {
      if (fadeHalfWidth <= 0.0) {
        return _omega >= _omegaCut ? 1.0 : 0.0;
      }
      const double x = Clamp(
        (_omega - (_omegaCut - fadeHalfWidth)) / (2.0 * fadeHalfWidth), 0.0, 1.0);
      return x * x * (3.0 - 2.0 * x);
    };

  // eta = Re sum a_k exp(i (k.x - omega t)), each a_k complex Gaussian with
  // E|a_k|^2 = 2 F(k) dk^2, so its variance is F(k) dk^2
  std::mt19937 rng(this->params.chopSeed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  this->chopAmplitude.assign(n * n, std::complex<double>(0.0, 0.0));
  this->chopOmega.assign(n * n, 0.0);
  double variance = 0.0;
  for (int row = 0; row < n; ++row) {
    const int my = row < n / 2 ? row : row - n;
    for (int col = 0; col < n; ++col) {
      const int mx = col < n / 2 ? col : col - n;
      // Draw for every mode, so changing the spectrum keeps the pattern
      const double g1 = gauss(rng);
      const double g2 = gauss(rng);
      // Skip the mean and the Nyquist modes, which have no partner at -k
      if (mx == -n / 2 || my == -n / 2 || (mx == 0 && my == 0)) {
        continue;
      }
      const double kx = mx * dk;
      const double ky = my * dk;
      const double k = std::hypot(kx, ky);
      const double omega = std::sqrt(kGravity * k);
      const int index = row * n + col;
      this->chopOmega[index] = omega;
      // S(omega, theta) d omega d theta = F(k) k dk d theta, d omega/dk = g/(2 omega)
      const double f = chopSpectrumAt(omega, std::atan2(ky, kx)) * crossover(omega) *
        kGravity / (2.0 * omega * k);
      const double modeVariance = f * dk * dk;
      if (modeVariance <= 0.0) {
        continue;
      }
      this->chopAmplitude[index] = std::sqrt(modeVariance) * std::complex<double>(g1, g2);
      variance += modeVariance;
    }
  }
  if (variance <= 0.0) {
    return;
  }
  this->chopSigma = std::sqrt(variance);
  this->chopOn = true;

  // Radix-2 FFT tables
  int bits = 0;
  while ((1 << bits) < n) {
    ++bits;
  }
  this->fftReverse.resize(n);
  for (int i = 0; i < n; ++i) {
    int r = 0;
    for (int b = 0; b < bits; ++b) {
      if ((i >> b) & 1) {
        r |= 1 << (bits - 1 - b);
      }
    }
    this->fftReverse[i] = r;
  }
  this->fftTwiddle.resize(n / 2);
  for (int i = 0; i < n / 2; ++i) {
    this->fftTwiddle[i] = std::complex<float>(std::polar(1.0, 2.0 * kPi * i / n));
  }
  for (auto & spectrum : this->chopSpectrum) {
    spectrum.assign(n * n, std::complex<float>(0.0f, 0.0f));
  }
  this->chopTile.assign(5 * n * n, 0.0f);
}

//////////////////////////////////////////////////
void WaveSurfaceField::Follow(double _x, double _y)
{
  const double limit = std::max(0.5 * this->fineHalf, this->fineSpacing);
  if (this->haveCenter &&
    std::abs(_x - this->centerX) <= limit && std::abs(_y - this->centerY) <= limit)
  {
    return;
  }
  this->centerX = std::round(_x / this->fineSpacing) * this->fineSpacing;
  this->centerY = std::round(_y / this->fineSpacing) * this->fineSpacing;
  this->haveCenter = true;
}

//////////////////////////////////////////////////
void WaveSurfaceField::ComputeLines(double _t)
{
  if (this->lines.empty()) {
    return;
  }

  // Split each line into chunks so the threads share the work whether there
  // is one direction or many
  struct Task
  {
    std::size_t line;
    std::size_t begin;
    std::size_t end;
  };
  std::vector<Task> tasks;
  const std::size_t chunksPerLine = std::max<std::size_t>(
    1, (2 * this->numThreads + this->lines.size() - 1) / this->lines.size());
  for (std::size_t l = 0; l < this->lines.size(); ++l) {
    Line & line = this->lines[l];
    line.xi0 = this->centerX * line.cosBeta + this->centerY * line.sinBeta - line.halfSpan;
    const std::size_t chunk = std::max<std::size_t>(
      64, (line.numSamples + chunksPerLine - 1) / chunksPerLine);
    for (std::size_t begin = 0; begin < line.numSamples; begin += chunk) {
      tasks.push_back({l, begin, std::min(begin + chunk, line.numSamples)});
    }
  }

  this->pool->Run(
    static_cast<int>(tasks.size()), [this, &tasks, _t](int _task)
    {
      const Task & task = tasks[_task];
      Line & line = this->lines[task.line];
      const std::size_t n = line.A.size();

      // Each component as a phasor A exp(i (k xi - omega t + phase)),
      // stepped along the line by exp(i k dxi)
      std::vector<double> re(n);
      std::vector<double> im(n);
      const double xi = line.xi0 + static_cast<double>(task.begin) * line.dxi;
      for (std::size_t c = 0; c < n; ++c) {
        const double theta = line.k[c] * xi - line.omega[c] * _t + line.phase[c];
        re[c] = line.A[c] * std::cos(theta);
        im[c] = line.A[c] * std::sin(theta);
      }
      const double * k = line.k.data();
      const double * stepRe = line.stepRe.data();
      const double * stepIm = line.stepIm.data();

      for (std::size_t j = task.begin; j < task.end; ++j) {
        // Four partial sums keep the additions from serializing
        double value[4] = {0.0, 0.0, 0.0, 0.0};
        double slope[4] = {0.0, 0.0, 0.0, 0.0};
        std::size_t c = 0;
        for (; c + 4 <= n; c += 4) {
          for (int u = 0; u < 4; ++u) {
            const double r = re[c + u];
            const double i = im[c + u];
            value[u] += r;
            slope[u] += k[c + u] * i;
            re[c + u] = r * stepRe[c + u] - i * stepIm[c + u];
            im[c + u] = r * stepIm[c + u] + i * stepRe[c + u];
          }
        }
        for (; c < n; ++c) {
          const double r = re[c];
          const double i = im[c];
          value[0] += r;
          slope[0] += k[c] * i;
          re[c] = r * stepRe[c] - i * stepIm[c];
          im[c] = r * stepIm[c] + i * stepRe[c];
        }
        line.value[j] = (value[0] + value[1]) + (value[2] + value[3]);
        // d/dxi of A cos(theta) is -k A sin(theta)
        line.slope[j] = -((slope[0] + slope[1]) + (slope[2] + slope[3]));
      }
    });
}

//////////////////////////////////////////////////
double WaveSurfaceField::WavesAt(
  double _x, double _y, double & _slopeX, double & _slopeY) const
{
  double eta = 0.0;
  _slopeX = 0.0;
  _slopeY = 0.0;
  for (const Line & line : this->lines) {
    const double xi = _x * line.cosBeta + _y * line.sinBeta;
    const double u = Clamp(
      (xi - line.xi0) / line.dxi, 0.0, static_cast<double>(line.numSamples - 1));
    const std::size_t j = std::min(static_cast<std::size_t>(u), line.numSamples - 2);
    const double f = u - static_cast<double>(j);
    const double f2 = f * f;
    const double f3 = f2 * f;
    const double p0 = line.value[j];
    const double p1 = line.value[j + 1];
    const double m0 = line.slope[j] * line.dxi;
    const double m1 = line.slope[j + 1] * line.dxi;
    // Cubic Hermite and its derivative
    const double value = (2.0 * f3 - 3.0 * f2 + 1.0) * p0 + (f3 - 2.0 * f2 + f) * m0 +
      (-2.0 * f3 + 3.0 * f2) * p1 + (f3 - f2) * m1;
    const double slope = ((6.0 * f2 - 6.0 * f) * (p0 - p1) + (3.0 * f2 - 4.0 * f + 1.0) * m0 +
      (3.0 * f2 - 2.0 * f) * m1) / line.dxi;
    eta += value;
    _slopeX += line.cosBeta * slope;
    _slopeY += line.sinBeta * slope;
  }
  return eta;
}

//////////////////////////////////////////////////
void WaveSurfaceField::ComputeChop(double _t)
{
  const int n = this->chopN;
  const double dk = 2.0 * kPi / this->params.chopPatchSize;
  auto & spectrum0 = this->chopSpectrum[0];
  auto & spectrum1 = this->chopSpectrum[1];
  auto & spectrum2 = this->chopSpectrum[2];

  // Spectra of the five real fields, two per complex FFT:
  //   0: eta + i d eta/dx,  1: d eta/dy + i dx,  2: dy
  // h(k, t) = (a_k exp(-i omega t) + conj(a_-k) exp(i omega t)) / 2 is
  // Hermitian, and so is every i k h, so each field comes out real.
  // The horizontal displacement i (k/|k|) h moves points toward the crests,
  // which sharpens them.
  this->pool->Run(
    n, [&](int _row)
    {
      const int my = _row < n / 2 ? _row : _row - n;
      const int negRow = (n - _row) % n;
      const std::complex<double> i(0.0, 1.0);
      for (int col = 0; col < n; ++col) {
        const int index = _row * n + col;
        const double omega = this->chopOmega[index];
        if (omega == 0.0) {
          spectrum0[index] = spectrum1[index] = spectrum2[index] = std::complex<float>();
          continue;
        }
        const int mx = col < n / 2 ? col : col - n;
        const int negIndex = negRow * n + (n - col) % n;
        const std::complex<double> rot = std::polar(1.0, -omega * _t);
        const std::complex<double> h = 0.5 *
        (this->chopAmplitude[index] * rot + std::conj(this->chopAmplitude[negIndex] * rot));
        const double kx = mx * dk;
        const double ky = my * dk;
        const double k = std::hypot(kx, ky);
        const std::complex<double> slopeX = i * kx * h;
        const std::complex<double> slopeY = i * ky * h;
        const std::complex<double> shiftX = i * (kx / k) * h;
        const std::complex<double> shiftY = i * (ky / k) * h;
        spectrum0[index] = std::complex<float>(h + i * slopeX);
        spectrum1[index] = std::complex<float>(slopeY + i * shiftX);
        spectrum2[index] = std::complex<float>(shiftY);
      }
    });

  for (auto & spectrum : this->chopSpectrum) {
    this->InverseFft2d(spectrum);
  }

  this->pool->Run(
    n, [&](int _row)
    {
      for (int col = 0; col < n; ++col) {
        const int index = _row * n + col;
        float * cell = &this->chopTile[5 * index];
        cell[0] = spectrum0[index].real();
        cell[1] = spectrum0[index].imag();
        cell[2] = spectrum1[index].real();
        cell[3] = spectrum1[index].imag();
        cell[4] = spectrum2[index].real();
      }
    });
}

//////////////////////////////////////////////////
void WaveSurfaceField::InverseFft(std::complex<float> * _data) const
{
  // Iterative radix-2, unnormalized, exp(+i ...)
  const int n = this->chopN;
  for (int i = 0; i < n; ++i) {
    const int r = this->fftReverse[i];
    if (i < r) {
      std::swap(_data[i], _data[r]);
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    const int halfLen = len >> 1;
    const int step = n / len;
    for (int i = 0; i < n; i += len) {
      for (int j = 0; j < halfLen; ++j) {
        const std::complex<float> w = this->fftTwiddle[j * step];
        const std::complex<float> u = _data[i + j];
        const std::complex<float> b = _data[i + j + halfLen];
        const float vr = b.real() * w.real() - b.imag() * w.imag();
        const float vi = b.real() * w.imag() + b.imag() * w.real();
        _data[i + j] = std::complex<float>(u.real() + vr, u.imag() + vi);
        _data[i + j + halfLen] = std::complex<float>(u.real() - vr, u.imag() - vi);
      }
    }
  }
}

//////////////////////////////////////////////////
void WaveSurfaceField::InverseFft2d(std::vector<std::complex<float>> & _data)
{
  const int n = this->chopN;
  this->pool->Run(n, [this, &_data, n](int _row) {this->InverseFft(&_data[_row * n]);});
  this->pool->Run(
    n, [this, &_data, n](int _col)
    {
      std::vector<std::complex<float>> column(n);
      for (int row = 0; row < n; ++row) {
        column[row] = _data[row * n + _col];
      }
      this->InverseFft(column.data());
      for (int row = 0; row < n; ++row) {
        _data[row * n + _col] = column[row];
      }
    });
}

//////////////////////////////////////////////////
void WaveSurfaceField::SampleChop(double _x, double _y, float _out[5]) const
{
  // Bilinear, periodic
  const int n = this->chopN;
  const int mask = n - 1;
  const double u = _x / this->chopCell;
  const double v = _y / this->chopCell;
  const double floorU = std::floor(u);
  const double floorV = std::floor(v);
  const float ax = static_cast<float>(u - floorU);
  const float ay = static_cast<float>(v - floorV);
  const int col0 = static_cast<int>(static_cast<std::int64_t>(floorU) & mask);
  const int row0 = static_cast<int>(static_cast<std::int64_t>(floorV) & mask);
  const int col1 = (col0 + 1) & mask;
  const int row1 = (row0 + 1) & mask;
  const float * c00 = &this->chopTile[5 * (row0 * n + col0)];
  const float * c10 = &this->chopTile[5 * (row0 * n + col1)];
  const float * c01 = &this->chopTile[5 * (row1 * n + col0)];
  const float * c11 = &this->chopTile[5 * (row1 * n + col1)];
  const float w00 = (1.0f - ax) * (1.0f - ay);
  const float w10 = ax * (1.0f - ay);
  const float w01 = (1.0f - ax) * ay;
  const float w11 = ax * ay;
  for (int f = 0; f < 5; ++f) {
    _out[f] = w00 * c00[f] + w10 * c10[f] + w01 * c01[f] + w11 * c11[f];
  }
}

//////////////////////////////////////////////////
void WaveSurfaceField::Compute(double _t, std::vector<float> & _vertices)
{
  this->ComputeLines(_t);
  if (this->chopOn) {
    this->ComputeChop(_t);
  }

  const int side = static_cast<int>(this->offsets.size());
  _vertices.resize(static_cast<std::size_t>(side) * side * kFloatsPerVertex);
  const double choppiness = this->params.choppiness;
  const double uvScale = 1.0 / std::max(this->params.chopPatchSize, 1e-3);
  const double inner = this->meshHalf + 1e-6;

  this->pool->Run(
    side, [&](int _row)
    {
      const double y = this->centerY + this->offsets[_row];
      const float rowWeight = this->chopWeight[_row];
      const bool skirtRow = std::abs(this->offsets[_row]) > inner;
      float * out = &_vertices[static_cast<std::size_t>(_row) * side * kFloatsPerVertex];
      for (int col = 0; col < side; ++col, out += kFloatsPerVertex) {
        const double x = this->centerX + this->offsets[col];
        double eta = 0.0;
        double slopeX = 0.0;
        double slopeY = 0.0;
        double shiftX = 0.0;
        double shiftY = 0.0;
        // The skirt stays flat
        const double edge = this->edgeWeight[_row] * this->edgeWeight[col];
        if (!skirtRow && edge > 0.0) {
          eta = edge * this->WavesAt(x, y, slopeX, slopeY);
          slopeX *= edge;
          slopeY *= edge;
          const float weight = std::min(rowWeight, this->chopWeight[col]);
          if (this->chopOn && weight > 0.0f) {
            float chop[5];
            this->SampleChop(x, y, chop);
            eta += weight * chop[0];
            shiftX = choppiness * weight * chop[3];
            shiftY = choppiness * weight * chop[4];
            // With a normal map, the chop's slopes come from it instead
            if (!this->normalMapOn) {
              slopeX += weight * chop[1];
              slopeY += weight * chop[2];
            }
          }
        }
        const double invLength = 1.0 / std::sqrt(slopeX * slopeX + slopeY * slopeY + 1.0);
        const double invTangent = 1.0 / std::sqrt(slopeX * slopeX + 1.0);
        out[0] = static_cast<float>(this->offsets[col] + shiftX);
        out[1] = static_cast<float>(this->offsets[_row] + shiftY);
        out[2] = static_cast<float>(eta);
        out[3] = static_cast<float>(-slopeX * invLength);
        out[4] = static_cast<float>(-slopeY * invLength);
        out[5] = static_cast<float>(invLength);
        // Tangent along +x, so the normal map's x and y are the world's
        out[6] = static_cast<float>(invTangent);
        out[7] = 0.0f;
        out[8] = static_cast<float>(slopeX * invTangent);
        // Fixed to the world, so the normal map lines up with the chop
        out[9] = static_cast<float>(x * uvScale);
        out[10] = static_cast<float>(y * uvScale);
      }
    });

  if (this->normalMapOn) {
    this->ComputeNormalMap();
  }
}

//////////////////////////////////////////////////
void WaveSurfaceField::ComputeNormalMap()
{
  // Level 0 from the chop slopes; each further level averages the slopes of
  // the one above, so distant chop blends instead of aliasing
  const int n = this->chopN;
  std::vector<float> slopeX(static_cast<std::size_t>(n) * n);
  std::vector<float> slopeY(slopeX.size());
  for (std::size_t i = 0; i < slopeX.size(); ++i) {
    slopeX[i] = this->chopTile[5 * i + 1];
    slopeY[i] = this->chopTile[5 * i + 2];
  }
  int levels = 1;
  while ((n >> levels) > 0) {
    ++levels;
  }
  this->normalLevels.resize(levels);
  for (int level = 0; level < levels; ++level) {
    const int size = n >> level;
    auto & texels = this->normalLevels[level];
    texels.resize(2 * static_cast<std::size_t>(size) * size);
    for (int i = 0; i < size * size; ++i) {
      const float invLength = 1.0f /
        std::sqrt(slopeX[i] * slopeX[i] + slopeY[i] * slopeY[i] + 1.0f);
      texels[2 * i] = static_cast<std::int8_t>(std::lround(-slopeX[i] * invLength * 127.0f));
      texels[2 * i + 1] = static_cast<std::int8_t>(std::lround(-slopeY[i] * invLength * 127.0f));
    }
    if (size == 1) {
      break;
    }
    const int half = size / 2;
    for (int row = 0; row < half; ++row) {
      for (int col = 0; col < half; ++col) {
        const int a = 2 * row * size + 2 * col;
        const int b = a + size;
        slopeX[row * half + col] =
          0.25f * (slopeX[a] + slopeX[a + 1] + slopeX[b] + slopeX[b + 1]);
        slopeY[row * half + col] =
          0.25f * (slopeY[a] + slopeY[a + 1] + slopeY[b] + slopeY[b + 1]);
      }
    }
  }
}

//////////////////////////////////////////////////
int WaveSurfaceField::VerticesPerSide() const
{
  return static_cast<int>(this->offsets.size());
}

//////////////////////////////////////////////////
const std::vector<std::uint32_t> & WaveSurfaceField::Indices() const
{
  return this->indices;
}

//////////////////////////////////////////////////
double WaveSurfaceField::CenterX() const
{
  return this->centerX;
}

//////////////////////////////////////////////////
double WaveSurfaceField::CenterY() const
{
  return this->centerY;
}

//////////////////////////////////////////////////
double WaveSurfaceField::BoundHalfWidth() const
{
  return this->boundHalfWidth;
}

//////////////////////////////////////////////////
double WaveSurfaceField::BoundHalfHeight() const
{
  return this->boundHalfHeight;
}

//////////////////////////////////////////////////
const std::string & WaveSurfaceField::Summary() const
{
  return this->summary;
}

//////////////////////////////////////////////////
int WaveSurfaceField::NormalMapSize() const
{
  return this->normalMapOn ? this->chopN : 0;
}

//////////////////////////////////////////////////
const std::vector<std::vector<std::int8_t>> & WaveSurfaceField::NormalMapLevels() const
{
  return this->normalLevels;
}
}  // namespace inc_wave_surface
}  // namespace buoy_gazebo
