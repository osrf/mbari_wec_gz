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


// Interpolation for efficiency maps
#include <simple_interp/interp1d.hpp>

#include <unsupported/Eigen/NonLinearOptimization>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ElectroHydraulicState.hpp"
#include "WindingCurrentTarget.hpp"


/////////////////////////////////////////////////////
// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>
    JacobianType;

  const int m_inputs, m_values;

  Functor()
  : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
  {
  }
  Functor(int inputs, int values)
  : m_inputs(inputs), m_values(values)
  {
  }

  int inputs() const
  {
    return m_inputs;
  }
  int values() const
  {
    return m_values;
  }

  // you should define that in the subclass :
  // void operator() (const InputType& x, ValueType* v, JacobianType* _j=0)
  // const;
};


int sgn(double v)
{
  if (v < 0) {return -1;}
  if (v > 0) {return 1;}
  return 0;
}

struct ElectroHydraulicSoln : Functor<double>
{
public:
  const double PressReliefSetPoint = 2850.0;       // psi
  // ~50GPM/600psi ~= .33472 in^3/psi -> Relief valve flow
  // above setpoint, From SUN RPECLAN data sheet
  const double ReliefValveFlowPerPSI = 30.0 / 600.0;     //50.0 / 600.0;
  const double CubicInchesPerGallon = 231.0;
  const double SecondsPerMinute = 60.0;

  simple_interp::Interp1d hyd_eff_v, hyd_eff_m;

  // Class that computes Target Winding Current based on RPM, Scale Factor,
  // limits, etc..
  mutable WindingCurrentTarget I_Wind;
  mutable double BusPower;
  double Q;
  /// \brief Pump/Motor Displacement per Revolution
  double HydMotorDisp;
  double VBattEMF;       // Battery internal EMF voltage
  double Ri;       // Battery internal resistance

private:
  static constexpr double RPM_TO_RAD_PER_SEC{2.0 * M_PI / 60.0};
  static constexpr double NM_PER_INLB{0.112984829};

  static const std::vector<double> Peff;       // psi
  static const std::vector<double> Neff;       // rpm

  static const std::vector<double> Eff_V;       // volumetric efficiency
  static const std::vector<double> Eff_M;       // mechanical efficiency

public:
  ElectroHydraulicSoln()
  : Functor<double>(3, 3),
    // Set HydraulicMotor Volumetric & Mechanical Efficiency
    hyd_eff_v(Peff, Eff_V), hyd_eff_m(Neff, Eff_M)
  {
  }

  // Friction loss is characterized in 2022 PTO simulation paper
  // Friction loss is a function of RPM
  // Units of power returned
  double MotorDriveFrictionLoss(double N) const
  {
    double tau_c = 0.1;             // N-m
    double k_v = .06 / 1000;             // N-m/RPM
    double k_th = 20;
    return fabs((tau_c * tanh(2 * M_PI * N / 60 / k_th) + k_v * N / 1000) * N);
  }

  // Switching Loss is from measurements as a function of bus voltage.
  // ~ 10% of Voltage in Watts...
  // Units of power returned
  double MotorDriveSwitchingLoss(double V) const
  {
    double k = 0.1;             // W/Volt  (magic units!)
    return k * fabs(V);
  }

  // Winding ISquaredR Losses,
  // Units of power returned
  double MotorDriveISquaredRLoss(double I) const
  {
    double R_w = 1.8;             // Ohms
    return R_w * I * I;
  }

  // x[0] = RPM
  // x[1] = Pressure (psi)
  // x[2] = Bus Voltage (Volts)
  int operator()(const Eigen::VectorXd & x, Eigen::VectorXd & fvec) const
  {

//    std::cout << "In Soln(): " << x[0] << "  " << x[1] << "  "  << x[2] << std::endl;
    const int n = x.size();
    assert(fvec.size() == n);

    const double rpm = fabs(x[0U]);
    const double pressure = fabs(x[1U]);
    //const double eff_m = this->hyd_eff_m.eval(rpm);
    //const double eff_v = this->hyd_eff_v.eval(pressure);
    const double eff_m = 1.0 - (.1 / 6000.0) * rpm;
    const double eff_v = 1.0 - (.1 / 3500.0) * pressure;

    double WindCurr = this->I_Wind(x[0U]);
    // 1.375 fudge factor required to match experiments, not yet sure why.
    const double T_applied =
      1.375 * this->I_Wind.TorqueConstantInLbPerAmp * WindCurr;

    double ShaftMechPower = T_applied * NM_PER_INLB * x[0U] * RPM_TO_RAD_PER_SEC;
    BusPower = -ShaftMechPower - (
      MotorDriveFrictionLoss(x[0U]) +
      MotorDriveSwitchingLoss(x[2U]) +
      MotorDriveISquaredRLoss(WindCurr));
    double Q_Relief = 0;
    if (x[1U] < -PressReliefSetPoint) {                  // Pressure relief is a one wave valve,
      // relieves when lower pressure is higher
      // than upper (resisting extension)
      Q_Relief = (x[1U] + PressReliefSetPoint) * ReliefValveFlowPerPSI *
        CubicInchesPerGallon / SecondsPerMinute;
    }
//		Q_Relief = 0;
    double Q_Motor = this->Q - Q_Relief;
    double Q = x[0U] * this->HydMotorDisp / SecondsPerMinute;
    double Q_Leak = (1.0 - eff_v) * std::max(fabs(Q_Motor), fabs(Q)) * sgn(x[1]);

    double T_Fluid = x[1U] * this->HydMotorDisp / (2.0 * M_PI);
    double T_Friction = -(1.0 - eff_m) * std::max(fabs(T_applied), fabs(T_Fluid)) * sgn(x[0]);

    fvec[0U] = Q_Motor - Q_Leak - Q;
    fvec[1U] = T_applied + T_Friction + T_Fluid;
    fvec[2U] = BusPower - (x[2U] - VBattEMF) * x[2U] / this->Ri;

    return 0;
  }

#if 0
  int df(const VectorXd & x, MatrixXd & fjac)
  {
    const int n = x.size();
    assert(fjac.rows() == n);
    assert(fjac.cols() == n);
    for (int k = 0; k < n; k++) {
      for (int j = 0; j < n; j++) {
        fjac(k, j) = 0.;
      }
      fjac(k, k) = 3. - 4. * x[k];
      if (k) {fjac(k, k - 1) = -1.;}
      if (k != n - 1) {fjac(k, k + 1) = -2.;}
    }
    return 0;
  }
#endif
};
#if 1
const std::vector<double> ElectroHydraulicSoln::Peff{
  0.0, 145.0, 290.0, 435.0, 580.0, 725.0, 870.0,
  1015.0, 1160.0, 1305.0, 1450.0, 1595.0, 1740.0, 1885.0,
  2030.0, 2175.0, 2320.0, 2465.0, 2610.0, 2755.0, 3500.0, 10000.0};

const std::vector<double> ElectroHydraulicSoln::Eff_V{
  1.0000, 0.980, 0.97, 0.960, 0.9520, 0.950, 0.949, 0.9480,
  0.9470, 0.946, 0.9450, 0.9440, 0.9430, 0.9420, 0.9410, 0.9400,
  0.9390, 0.9380, 0.9370, 0.9350, 0.9100, .6000};


const std::vector<double> ElectroHydraulicSoln::Neff{
  0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0,
  800.0, 900.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0,
  4000.0, 4500.0, 5000.0, 5500.0, 6000.0, 15000.0};

const std::vector<double> ElectroHydraulicSoln::Eff_M{
  1.0, 0.950, 0.9460, 0.9450, 0.9440, 0.9430, 0.9420, 0.9410,
  0.9400, 0.9390, 0.9380, 0.9370, 0.9360, 0.9370, 0.9360, 0.9350,
  0.9320, 0.9300, 0.9200, 0.9100, 0.9000, 0.8400};
#endif
#if 0
const std::vector<double> ElectroHydraulicSoln::Peff{
  0.0, 145.0, 290.0, 435.0, 580.0, 725.0, 870.0,
  1015.0, 1160.0, 1305.0, 1450.0, 1595.0, 1740.0, 1885.0,
  2030.0, 2175.0, 2320.0, 2465.0, 2610.0, 2755.0, 2900.0};

const std::vector<double> ElectroHydraulicSoln::Neff{
  0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0,
  800.0, 900.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0,
  4000.0, 4500.0, 5000.0, 5500.0, 6000.0, 15000.0};

const std::vector<double> ElectroHydraulicSoln::Eff_V{
  1.0000, 0.8720, 0.9240, 0.9480, 0.9520, 0.9620, 0.9660, 0.9660,
  0.9720, 0.9760, 0.9760, 0.9800, 0.9800, 0.9800, 0.9800, 0.9800,
  0.9800, 0.9800, 0.9800, 0.9800, 0.9800};

const std::vector<double> ElectroHydraulicSoln::Eff_M{
  1.0, 0.9360, 0.9420, 0.9460, 0.9460, 0.9460, 0.9460, 0.9460,
  0.9460, 0.9460, 0.9460, 0.9500, 0.9500, 0.9500, 0.9500, 0.9400,
  0.9300, 0.9300, 0.9200, 0.9100, 0.9000, 0.8400};
#endif

struct EffV
{
  EffV() = default;

  std::vector<double> df(const double & rpm, const double & pressure) const
  {
    double sech_ = 1.0 / cosh(2e-6 * (rpm + 55.0) * (pressure - 5000.0));

    // rpm
    double dfdx0 = 2.667e-11 * (pressure - 74268.6) * (pressure - 5000.0) * sech_ * sech_;

    // pressure
    double dfdx1 = 1.333e-5 * tanh(2e-6 * (rpm + 55.0) * (pressure - 5000.0)) +
      2.667e-11 * (rpm + 55.0) * (pressure - 74268.6) * sech_ * sech_;

    return std::vector<double>{dfdx0, dfdx1};
  }

  double operator()(const double & rpm, const double & pressure) const
  {
    return std::max(
      0.5,
      (-1.333e-5 * pressure + 0.99) * tanh((-2e-6 * pressure + 0.01) * (rpm + 55.0)));
  }
};


struct EffM
{
  EffM() = default;

  std::vector<double> df(const double & rpm, const double & pressure) const
  {
    double sech_ = 1.0 / cosh(8e-8 * (rpm - 37500.0) * (pressure + 34.0));

    // rpm
    double dfdx0 = (4.2664e-13 * rpm - 7.84e-8) * (pressure + 34.0) * sech_ * sech_ -
      5.333e-6 * tanh((-8e-8 * rpm + 0.003) * (pressure + 34.0));

    // pressure
    double dfdx1 = 4.2664e-13 * (rpm - 183761.0) * (rpm - 37500.0) * sech_ * sech_;

    return std::vector<double>{dfdx0, dfdx1};
  }

  double operator()(const double & rpm, const double & pressure) const
  {
    if (pressure <= 10.0) {
      return (2.5871e-14 * rpm + 7.9528e-9) * exp((-3.9e-6 * rpm + 1.25742) * pressure) + 0.1;
    }
    return std::max(
      0.1,
      (-5.333e-6 * rpm + 0.98) * tanh((-8e-8 * rpm + 0.003) * (pressure + 34.0)));
  }
};

#endif  // ELECTROHYDRAULICPTO__ELECTROHYDRAULICSOLN_HPP_
