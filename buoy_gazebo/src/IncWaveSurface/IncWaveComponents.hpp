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

#ifndef INCWAVESURFACE__INCWAVECOMPONENTS_HPP_
#define INCWAVESURFACE__INCWAVECOMPONENTS_HPP_

#include <gz/msgs/double_v.pb.h>

#include <cstddef>
#include <string>
#include <vector>


namespace buoy_gazebo
{
/// \brief Wave components of one direction entry of a LinearIncidentWave.
///
/// eta = sum_i A_i cos(k_i (x cos(beta) + y sin(beta)) - omega_i t + phase_i)
struct IncWaveEntry
{
  /// \brief Propagation direction, rad CCW from +x (where waves travel to)
  double beta{0.0};

  /// \brief Amplitude (m), frequency (rad/s), wavenumber (rad/m) and phase
  /// (rad) of each component
  std::vector<double> A;
  std::vector<double> omega;
  std::vector<double> k;
  std::vector<double> phase;
};

/// \brief The incident wave field as FreeSurfaceHydrodynamics generated it.
/// Every wave type has this form, so this covers all of them.
struct IncWaveComponents
{
  /// \brief Highest frequency FreeSurfaceHydrodynamics generates (Hz)
  double maxFreqHz{0.0};

  std::vector<IncWaveEntry> entries;
};

/// \brief Service the IncWaveSurface plugin serves the components on
inline std::string IncWaveComponentsService(const std::string & _worldName)
{
  return "/world/" + _worldName + "/inc_wave_components";
}

// Packed layout:
//   [version, maxFreqHz, number of entries,
//    then for each entry: beta, n, A[n], omega[n], k[n], phase[n]]
static constexpr double kIncWaveComponentsVersion = 1.0;

/// \brief Pack the components into a message
inline void PackIncWaveComponents(
  const IncWaveComponents & _in, gz::msgs::Double_V & _out)
{
  _out.clear_data();
  _out.add_data(kIncWaveComponentsVersion);
  _out.add_data(_in.maxFreqHz);
  _out.add_data(static_cast<double>(_in.entries.size()));
  for (const auto & entry : _in.entries) {
    const std::size_t n = entry.A.size();
    _out.add_data(entry.beta);
    _out.add_data(static_cast<double>(n));
    for (const auto * values : {&entry.A, &entry.omega, &entry.k, &entry.phase}) {
      for (std::size_t i = 0; i < n; ++i) {
        _out.add_data((*values)[i]);
      }
    }
  }
}

/// \brief Unpack components packed by PackIncWaveComponents
/// \return False if the message is malformed or of another version
inline bool UnpackIncWaveComponents(
  const gz::msgs::Double_V & _in, IncWaveComponents & _out)
{
  const int size = _in.data_size();
  int p = 0;
  if (size < 3 || _in.data(0) != kIncWaveComponentsVersion) {
    return false;
  }
  _out.maxFreqHz = _in.data(1);
  const double numEntries = _in.data(2);
  p = 3;
  if (numEntries < 0.0 || numEntries > size) {
    return false;
  }
  _out.entries.assign(static_cast<std::size_t>(numEntries), IncWaveEntry{});
  for (auto & entry : _out.entries) {
    if (p + 2 > size) {
      return false;
    }
    entry.beta = _in.data(p++);
    const double n = _in.data(p++);
    if (n < 0.0 || p + 4.0 * n > size) {
      return false;
    }
    for (auto * values : {&entry.A, &entry.omega, &entry.k, &entry.phase}) {
      values->resize(static_cast<std::size_t>(n));
      for (auto & value : *values) {
        value = _in.data(p++);
      }
    }
  }
  return p == size;
}
}  // namespace buoy_gazebo

#endif  // INCWAVESURFACE__INCWAVECOMPONENTS_HPP_
