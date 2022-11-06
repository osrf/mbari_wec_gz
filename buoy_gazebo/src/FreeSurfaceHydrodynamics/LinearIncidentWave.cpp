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


#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>

#include <Eigen/Dense>
#include "LinearIncidentWave.hpp"

/// \brief Constructor, defaults to monotchromatic wave and default gravity and density
LinearIncidentWave::LinearIncidentWave()
{
  m_grav = 9.81;
  m_rho = 1025;
}

/// \brief Select PM-Spectrum (default num of phases)
void LinearIncidentWave::SetToPiersonMoskowitzSpectrum(double Hs, double beta)
{
  SetToPiersonMoskowitzSpectrum(Hs, beta, DEFAULT_N_PHASES);
}

/// \brief Select PM-Spectrum (set num of phases)
void LinearIncidentWave::SetToPiersonMoskowitzSpectrum(double Hs, double beta, int n_phases)
{
  m_SpectrumType = WaveSpectrumType::PiersonMoskowitz;
  m_beta = beta;
  m_Hs = Hs;


  m_omega.resize(n_phases);
  m_k.resize(n_phases);
  m_phases.resize(n_phases);
  m_Spectrum.resize(n_phases);
  m_A.resize(n_phases);

  double w0 = sqrt(.21 * m_grav / Hs);
  double a = 0.0081;
  double b = 0.74;
  std::srand(time(0));  // Initialize random number generator.

  double d_omega = MAX_FREQ * 2 * M_PI / n_phases;

  for (int i = 0; i < m_k.size(); i++) {
    m_omega(i) = d_omega * (i + 1);
    m_k(i) = m_omega(i) * m_omega(i) / m_grav;
    m_Spectrum(i) = (a * m_grav * m_grav / pow(m_omega(i), 5)) * exp(-b * pow(w0 / m_omega(i), 4));
    m_A(i) = sqrt(d_omega * 2 * m_Spectrum(i));  // Precompute components once here.
    m_phases(i) = (2 * M_PI * std::rand()) / RAND_MAX;
  }
}

/// \brief Select single frequency wave
void LinearIncidentWave::SetToMonoChromatic(double A, double T, double phase, double beta)
{
  m_SpectrumType = WaveSpectrumType::MonoChromatic;
  m_Hs = 2 * A;
  m_Tp = T;
  m_beta = beta;
  m_omega.resize(1);
  m_k.resize(1);
  m_phases.resize(1);
  m_Spectrum.resize(1);
  m_A.resize(1);
  m_phases(0) = phase;
  m_omega(0) = 2 * M_PI / T;
  m_k(0) = m_omega(0) * m_omega(0) / m_grav;
  m_A(0) = A;
}

std::ostream & operator<<(std::ostream & out, const LinearIncidentWave & IncWave)
{
  // Since operator<< is a friend of the LinearIncidentWave class, we can access members directly.
  switch (IncWave.m_SpectrumType) {
    case WaveSpectrumType::MonoChromatic:
      std::cout << "# IncidentWave Type = Mono-Chromatic" << std::endl;
      std::cout << "# Amplitude = " << IncWave.m_Hs / 2 << std::endl;
      std::cout << "# Period = " << IncWave.m_Tp << std::endl;
      std::cout << "# Num Phases = " << IncWave.m_Spectrum.size() << std::endl;
      std::cout << "# Wave Freq = " << IncWave.m_omega.transpose() << std::endl;
      std::cout << "# Wave Numbers = " << IncWave.m_k.transpose() << std::endl;
      std::cout << "# Phases = " << IncWave.m_phases.transpose() << std::endl;
      std::cout << "# Component Amplitudes = " << IncWave.m_A.transpose() << std::endl;
      break;
    case WaveSpectrumType::PiersonMoskowitz:
      std::cout << "# IncidentWave Type = Pierson Moskowitz" << std::endl;
      std::cout << "# Hs = " << IncWave.m_Hs << std::endl;
      std::cout << "# Tp = " << IncWave.m_Tp << std::endl;
      std::cout << "# Num Phases = " << IncWave.m_Spectrum.size() << std::endl;
      std::cout << "# Wave Freq = " << IncWave.m_omega.transpose() << std::endl;
      std::cout << "# Wave Numbers = " << IncWave.m_k.transpose() << std::endl;
      std::cout << "# Phases = " << IncWave.m_phases.transpose() << std::endl;
      std::cout << "# Spectrum = " << IncWave.m_Spectrum.transpose() << std::endl;
      std::cout << "# Component Amplitudes = " << IncWave.m_A.transpose() << std::endl;
      break;

    case WaveSpectrumType::UserSupplied:
      std::cout << "# IncidentWave Type = User Defined Spectrum";
      break;
  }
  return out;  // return std::ostream so we can chain calls to operator<<
}

double LinearIncidentWave::eta(double x, double y, double t)
{
  double xx = x * cos(m_beta) + y * sin(m_beta);

  double eta = 0;
  for (int i = 0; i < m_A.size(); i++) {
    eta = eta + m_A(i) * cos(m_k(i) * xx - m_omega(i) * t + m_phases(i));
  }
  return eta;

// Eigen::VectorXd temp;
// temp.array()  = (xx*m_k-t*m_omega+m_phases).array().sin();
// return -m_A.dot(temp);  //With -03, these two approaches are equally fast/slow
}

double LinearIncidentWave::etadot(double x, double y, double t)
{
  double xx = x * cos(m_beta) + y * sin(m_beta);

  double etadot = 0;
  for (int i = 0; i < m_A.size(); i++) {
    etadot = etadot + m_omega(i) * m_A(i) * sin(m_k(i) * xx - m_omega(i) * t + m_phases(i));
  }

  return etadot;
}
