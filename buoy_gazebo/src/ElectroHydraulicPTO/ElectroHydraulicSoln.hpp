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

#ifndef ELECTROHYDRAULICPTO__ELECTROHYDRAULICSOLN_HPP_
#define ELECTROHYDRAULICPTO__ELECTROHYDRAULICSOLN_HPP_

#include <stdio.h>

#include <unsupported/Eigen/NonLinearOptimization>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "ElectroHydraulicState.hpp"

// Interpolation library for efficiency maps
#include "JustInterp/JustInterp.hpp"
#include "WindingCurrentTarget.hpp"


/////////////////////////////////////////////////////
// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor()
  : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
  {
  }
  Functor(int inputs, int values)
  : m_inputs(inputs), m_values(values)
  {
  }

  int inputs() const {return m_inputs;}
  int values() const {return m_values;}

  // you should define that in the subclass :
  // void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};


struct ElectroHydraulicSoln : Functor<double>
{
public:
  JustInterp::TableInterpolator<double> hyd_eff_v;
  JustInterp::TableInterpolator<double> hyd_eff_m;
  JustInterp::LinearInterpolator<double> reliefValve;
  // Class that computes Target Winding Current based on RPM, Scale Factor, limits, etc..
  WindingCurrentTarget I_Wind;
  double Q;
  /// \brief Pump/Motor Displacement per Revolution
  double HydMotorDisp;

private:
  static constexpr double Pset{2000.0};  // 750;  // psi
  // ~50GPM/600psi ~= .33472 in^3/psi -> From SUN RPECLAN data sheet
  static constexpr double QPerP{(50.0*241.0 / 60.0) / 600.0};
  static const std::vector<double> Prelief;  // {0, Pset, Pset + 600.0};
  static const std::vector<double> Qrelief;  // {0, 0, QPerP*6000.0};

  // psi
  static const std::vector<double> Peff;

  static const std::vector<std::vector<double> > Neff;
  static const std::vector<std::vector<double> > eff_v;
  static const std::vector<std::vector<double> > eff_m;

public:
  ElectroHydraulicSoln()
  : Functor<double>(2, 2),
    // Set Pressure versus flow relationship for relief valve
    reliefValve{Prelief.size(), Prelief.data(), Qrelief.data()},
    // Set HydrualicMotor Volumetric Efficiency
    hyd_eff_v{Peff, Neff, eff_v},
    hyd_eff_m{Peff, Neff, eff_m}
  {
  }

  // x[0] = RPM
  // x[1] = Pressure (psi)
  int operator()(const Eigen::VectorXd & x, Eigen::VectorXd & fvec) const
  {
    const int n = x.size();
    assert(fvec.size() == n);
    // assert(this->hyd_eff_m.GetX()[0U] && "eff_m xData_ is bad");
    // assert(this->hyd_eff_m.GetY(0U)[0U] && "eff_m y is bad");
    // assert(this->hyd_eff_v.GetX()[0U] && "eff_v xData_ is bad");
    // assert(this->hyd_eff_v.GetY(0U)[0U] && "eff_v y is bad");
    const double eff_m = this->hyd_eff_m(fabs(x[1]), fabs(x[0]));
    const double eff_v = this->hyd_eff_v(fabs(x[1]), fabs(x[0]));

    // 1.375 fudge factor required to match experiments, not yet sure why.
    const double T_applied = 1.375 * this->I_Wind.TorqueConstantInLbPerAmp * this->I_Wind(x[0]);

    static constexpr double Pset = 2925;
    double QQ = this->Q;
    if (x[1] > Pset) {   // Extending
      QQ += (x[1] - Pset) * (50 * 241 / 60) / 600;
    }
    // QQ += this->reliefValve(x[1]);

    fvec[0] = x[0] - eff_v * 60.0 * QQ / this->HydMotorDisp;
    fvec[1] = x[1] - eff_m * T_applied / (this->HydMotorDisp / (2 * M_PI));

    return 0;
  }
};

const std::vector<double> ElectroHydraulicSoln::Prelief{0, Pset, Pset + 600.0};
const std::vector<double> ElectroHydraulicSoln::Qrelief{0, 0, QPerP*6000.0};

const std::vector<double> ElectroHydraulicSoln::Peff
{0, 145, 290, 435, 580, 725, 870, 1015, 1160, 1305, 1450, 2176, 2901};

const std::vector<std::vector<double> > ElectroHydraulicSoln::Neff
{
  {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 15000},  // 0Mpa/0psi
  {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 15000},  // 1Mpa/145psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 2Mpa/290psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 3Mpa/435psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 4Mpa/580psi
  {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 15000},  // 5Mpa/725psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 6Mpa/870psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 7Mpa/1015psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 8Mpa/1160psi
  {0, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 15000},  // 9Mpa/1305psi
  {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 15000},  // 10Mpa/1450psi
  {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 15000},  // 15Mpa/2176psi
  {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 15000}  // 20Mpa/2901psi
};
const std::vector<std::vector<double> > ElectroHydraulicSoln::eff_v
{
  // {0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10,
  //  0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10},  // 0Mpa/0psi
  {0.97, 0.97, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 0.99,
    0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99},  // 1Mpa/145psi
  {0.97, 0.97, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 0.99,
    0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99},  // 1Mpa/145psi
  {0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99},  // 2Mpa/290psi
  {0.98, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99},  // 3Mpa/435psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99},  // 4Mpa/580psi
  {0.90, 0.90, 0.94, 0.96, 0.96, 0.97, 0.97, 0.97, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98,
    0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99},  // 5Mpa/725psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.99, 0.98},  // 6Mpa/870psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98},  // 7Mpa/1015psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98},  // 8Mpa/1160psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98},  // 9Mpa/1305psi
  {0.83, 0.83, 0.90, 0.93, 0.94, 0.95, 0.96, 0.96, 0.96, 0.97, 0.97, 0.97, 0.98, 0.98, 0.98,
    0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98},  // 10Mpa/1450psi
  {0.77, 0.77, 0.87, 0.90, 0.92, 0.93, 0.94, 0.95, 0.95, 0.96, 0.96, 0.96, 0.96, 0.96, 0.96,
    0.96, 0.96, 0.96, 0.96, 0.96, 0.96, 0.96},  // 15Mpa/2176psi
  {0.72, 0.72, 0.83, 0.88, 0.90, 0.92, 0.93, 0.94, 0.94, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95,
    0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95}  // 20Mpa/2901psi
};
const std::vector<std::vector<double> > ElectroHydraulicSoln::eff_m
{
  // {0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10,
  //  0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10},  // 0Mpa/0psi
  {0.60, 0.60, 0.61, 0.62, 0.63, 0.64, 0.65, 0.65, 0.66, 0.66, 0.67, 0.79, 0.79, 0.78, 0.77,
    0.74, 0.71, 0.68, 0.64, 0.60, 0.55, 0.15},  // 1Mpa/145psi
  {0.60, 0.60, 0.61, 0.62, 0.63, 0.64, 0.65, 0.65, 0.66, 0.66, 0.67, 0.79, 0.79, 0.78, 0.77,
    0.74, 0.71, 0.68, 0.64, 0.60, 0.55, 0.15},  // 1Mpa/145psi
  {0.84, 0.84, 0.84, 0.84, 0.82, 0.81, 0.78, 0.76, 0.73, 0.70, 0.66, 0.35},  // 2Mpa/290psi
  {0.89, 0.89, 0.89, 0.89, 0.88, 0.87, 0.85, 0.84, 0.82, 0.80, 0.77, 0.55},  // 3Mpa/435psi
  {0.92, 0.92, 0.92, 0.92, 0.91, 0.90, 0.89, 0.88, 0.86, 0.84, 0.83, 0.70},  // 4Mpa/580psi
  {0.92, 0.92, 0.93, 0.93, 0.93, 0.93, 0.93, 0.93, 0.93, 0.93, 0.93, 0.94, 0.94, 0.93, 0.93,
    0.92, 0.91, 0.90, 0.89, 0.87, 0.86, 0.80},  // 5Mpa/725psi
  {0.95, 0.95, 0.95, 0.94, 0.94, 0.93, 0.92, 0.92, 0.91, 0.89, 0.88, 0.82},  // 6Mpa/870psi
  {0.95, 0.95, 0.95, 0.95, 0.95, 0.94, 0.93, 0.93, 0.92, 0.91, 0.90, 0.84},  // 7Mpa/1015psi
  {0.96, 0.96, 0.96, 0.96, 0.95, 0.95, 0.94, 0.94, 0.93, 0.92, 0.91, 0.85},  // 8Mpa/1160psi
  {0.96, 0.96, 0.96, 0.96, 0.96, 0.95, 0.95, 0.94, 0.93, 0.93, 0.92, 0.86},  // 9Mpa/1305psi
  {0.96, 0.96, 0.96, 0.97, 0.97, 0.97, 0.97, 0.97, 0.97, 0.97, 0.97, 0.97, 0.97, 0.97, 0.96,
    0.96, 0.95, 0.95, 0.94, 0.93, 0.93, 0.88},  // 10Mpa/1450psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.97, 0.97, 0.97, 0.96,
    0.96, 0.95, 0.95, 0.94, 0.93, 0.93, 0.89},  // 15Mpa/2176psi
  {0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.97, 0.97, 0.97, 0.96,
    0.96, 0.95, 0.95, 0.94, 0.93, 0.93, 0.90}  // 20Mpa/2901psi
};

#endif  // ELECTROHYDRAULICPTO__ELECTROHYDRAULICSOLN_HPP_
