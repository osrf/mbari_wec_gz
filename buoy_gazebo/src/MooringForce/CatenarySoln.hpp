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

#ifndef MOORINGFORCE__CATENARYSOLN_HPP_
#define MOORINGFORCE__CATENARYSOLN_HPP_

#include <unsupported/Eigen/NonLinearOptimization>

#include <cmath>

#include <ignition/common/Console.hh>

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
  // int operator() (const InputType& x, ValueType* v, JacobianType* _j=0)
  // const;
};

struct CatenaryFunction
{
public:
  /// \brief Catenary chain calculation.
  /// V: meters, vertical distance from buoy to anchor
  /// B: Meters, length of mooring chain lying on seafloor, start
  /// of catenary
  /// L: meters, total length of mooring chain
  /// Returns c. Catenary curve from B to H
  static double CatenaryCoefficient(double _V, double _B, double _L)
  {
    // Catenary coefficient c
    return (pow((_L - _B), 2) - pow(_V, 2)) / (2. * _V);
  }
};

struct CatenaryVSoln : Functor<double>
{
private:

   /// \brief Meters, vertical distance from buoy to anchor
   double V = 82.0;

   /// \brief Meters, length of mooring chain lying on seafloor, start
   double B = 0.0;

   /// \brief Meters, total length of mooring chain
   double L = 160.0;

public:

  CatenaryVSoln(double _V, double _B, double _L)
  : Functor<double>(1, 1),
    V(_V),
    B(_B),
    L(_L)
  {
  }

  double y(double _c, double _x)
  {
    return _c * cosh((_x - this->B) / _c) - _c;
  }

  double f(double _c, double _x)
  {
    return (this->y(_c, _x) - this->V);
  }

  // y_B
  // Take in x=H as initial guess
  // x: location on chain
  int operator()(const Eigen::VectorXd & x, Eigen::VectorXd & fvec) const
  {
    if (x.size() < 1) {
      ignerr << "Invalid input size for CatenaryVSoln::operator()" << std::endl;
      return -1;
    }

    // Catenary coefficient
    double c = CatenaryFunction::CatenaryCoefficient(
      this->V, this->B, this->L);

    // Catenary equation: y = lambda x: c * cosh((x - B) / c) - c
    double y = c * cosh((x[0] - this->B) / c) - c;;
    fvec.resize(x.size());
    fvec[0] = y - this->V;

    return 0;
  }
};

struct CatenaryHSoln : Functor<double>
{
private:
   /// \brief Meters, vertical distance from buoy to anchor
   double V = 82.0;

   /// \brief Meters, horizontal distance from buoy to anchor
   double H = 120.0;

   /// \brief Meters, total length of mooring chain
   double L = 160.0;

public:
  CatenaryHSoln(double _V, double _H, double _L)
  : Functor<double>(1, 1),
    V(_V),
    H(_H),
    L(_L)
  {
  }

  // Take in L - V - b as initial guess for B
  int operator()(const Eigen::VectorXd & B, Eigen::VectorXd & fvec) const
  {
    if (B.size() < 1) {
      ignerr << "Invalid input size for CatenaryHSoln::operator()" << std::endl;
      return -1;
    }

    // Catenary coefficient
    double c = CatenaryFunction::CatenaryCoefficient(
      this->V, B[0], this->L);

    // Initial guess x = H, for CatenaryVSoln
    // TODO Is this how to pass in H as initial guess for x?
    Eigen::VectorXd x{};
    x.resize(1);
    x[0] = this->H;

    CatenaryVSoln y_B = CatenaryVSoln(this->V, B[0], this->L);
    Eigen::HybridNonLinearSolver<CatenaryVSoln> vSolver(y_B);
    // Tolerance for error between two consecutive iterations
    vSolver.parameters.xtol = 0.0001;
    // Max number of calls to the function
    vSolver.parameters.maxfev = 1000;

    // Call the solver
    int solverInfo = vSolver.solveNumericalDiff(x);

    // Recalculate y_B(B[0]) using solution of x from solver
    fvec.resize(x.size());
    fvec[0] = y_B.f(c, x[0]) - this->H;

    igndbg << "VSolver solverInfo: " << solverInfo << " fvec[0]: " << fvec[0]
      << std::endl;

    return 0;
  }
};

#endif  // MOORINGFORCE__CATENARYSOLN_HPP_
