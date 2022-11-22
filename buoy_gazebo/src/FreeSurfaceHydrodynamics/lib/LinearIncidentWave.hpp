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


#ifndef FREESURFACEHYDRODYNAMICS__LIB__LINEARINCIDENTWAVE_HPP_
#define FREESURFACEHYDRODYNAMICS__LIB__LINEARINCIDENTWAVE_HPP_

#include <Eigen/Dense>
#include <vector>
#include "IncidentWave.hpp"

#define DEFAULT_N_PHASES 300
#define MAX_FREQ .3  // Hz

enum class WaveSpectrumType {MonoChromatic, PiersonMoskowitz, UserSupplied};

/**
\brief The IncidentWave class generates wave heights and fluid velocities at specified points and times.
The waves are specfied as Monochromatic, Pierson-Moskowitz, or by a supplied spectrum. The wave direction
(beta) is specified at the time of wave-type selection.  At this time the resulting waves are from a
single direction.

 \brief After initialization, member functions provide the wave height and fluid velocities at specified points in
space and time.

 \author Andrew Hamilton
*/

class LinearIncidentWave : public IncidentWave
{
public:
  LinearIncidentWave();
  LinearIncidentWave(double L, double g, double rho);
  friend std::ostream & operator<<(std::ostream & out, const LinearIncidentWave & IncWave);
  void SetToPiersonMoskowitzSpectrum(double Hs, double beta);
  void SetToPiersonMoskowitzSpectrum(double Hs, double Tp, double beta, int n_phases);
  void SetToMonoChromatic(double A, double T, double phase, double beta);
  double eta(double x, double y, double t);
  // Eigen::VectorXd eta(double x, double y, Eigen::VectorXd t);
  double etadot(double x, double y, double t);

public:
  WaveSpectrumType m_SpectrumType = WaveSpectrumType::MonoChromatic;
  double m_grav = 9.81;
  double m_rho = 1025;
  Eigen::VectorXd m_Spectrum;
  Eigen::VectorXd m_A;
  Eigen::VectorXd m_omega;
  Eigen::VectorXd m_k;
  Eigen::VectorXd m_phases;
  double m_beta = 0;
  double m_Hs = 1;
  double m_Tp = 10;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> fd_Pha_Xi;
};

#endif  // FREESURFACEHYDRODYNAMICS__LIB__LINEARINCIDENTWAVE_HPP_
