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

#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include <cmath>

#include <gz/common/Console.hh>

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
  /// \brief Scaling factor for catenary chain
  /// V: meters, vertical distance from buoy to anchor
  /// B: Meters, length of mooring chain lying on seafloor, start of catenary
  /// L: meters, total length of mooring chain
  /// Returns c, scaling factor, ratio of horizontal component of chain tension
  /// and weight of cable per unit length
  static double CatenaryScalingFactor(double _V, double _B, double _L)
  {
    // Scaling factor c
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

  // y_B
  // Take in x=H as initial guess
  // x: location on chain
  int operator()(const Eigen::VectorXd & x, Eigen::VectorXd & fvec) const
  {
    if (x.size() < 1) {
      ignerr << "Invalid input size for CatenaryVSoln::operator()" << std::endl;
      return -1;
    }

    // Scaling factor
    double c = CatenaryFunction::CatenaryScalingFactor(
      this->V, this->B, this->L);

    // Catenary equation: y = lambda x: c * cosh((x - B) / c) - c
    double y = c * cosh((x[0] - this->B) / c) - c;;
    // fvec.resize(x.size());
    // What we want to be 0, y(H) - V
    fvec[0] = y - this->V;

    // igndbg << " fvec[0] (solved y(H)~V): " << fvec[0] << std::endl;

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

  // Know 0 <= B < L - V. Take in B = L - V - b as initial guess
  int operator()(const Eigen::VectorXd & B, Eigen::VectorXd & fvec) const
  {
    if (B.size() < 1) {
      ignerr << "Invalid input size for CatenaryHSoln::operator()" << std::endl;
      return -1;
    }

    // Initial guess x = H, for CatenaryVSoln
    Eigen::VectorXd x{};
    x.resize(1);
    x[0] = this->H;

    // Look for the H for the given B, y_B(B) = H, and y(H)=V
    CatenaryVSoln y_B = CatenaryVSoln(this->V, B[0], this->L);
    Eigen::HybridNonLinearSolver<CatenaryVSoln> vSolver(y_B);
    // Tolerance for error between two consecutive iterations
    vSolver.parameters.xtol = 0.0001;
    // Max number of calls to the function
    vSolver.parameters.maxfev = 1000;
    vSolver.diag.setConstant(1, 1.0);
    vSolver.useExternalScaling = true;  // Improves solution stability dramatically.
    // Solver for x. Pass in initial guess.
    // Want y(H)=V, so y(H) - V = 0
    // After this, you have independent variable = H, dependent variable = V
    int solverInfo = vSolver.solveNumericalDiff(x);

    // Once have x (hopefully x=H)
    // fvec.resize(x.size());
    // Update H, which is the solution of x from solver
    fvec[0] = x[0] - this->H;

    // igndbg << "VSolver solverInfo: " << solverInfo
    //   << " fvec[0] (solved x~H): " << fvec[0] << std::endl;

    return 0;
  }
};

#endif  // MOORINGFORCE__CATENARYSOLN_HPP_
