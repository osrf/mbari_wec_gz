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

struct CatenaryHSoln : Functor<double>
{
private:
   /// \brief Meters, vertical distance from buoy to anchor
   double V = 82.0;

   /// \brief Meters, horizontal distance from buoy to anchor
   double H = 120.0;

   /// \brief Meters, total length of mooring chain
   double L = 160.0;

   /// \brief For performance, set to true to skip solving vor CatenaryVSoln.
   /// For accuracy, set to false.
   bool SKIP_V_SOLVER = true;

public:
  CatenaryHSoln(double _V, double _H, double _L)
  : Functor<double>(1, 1),
    V(_V),
    H(_H),
    L(_L)
  {
  }

  /// \brief Invert the catenary equation and solve for the horizontal input.
  ///
  /// \brief B length of chain on floor.
  double InverseCatenaryVSoln(double B) const
  {
    double c = CatenaryFunction::CatenaryScalingFactor(this->V, B, this->L);
    return c * std::acosh(this->V / c + 1.0) + B;
  }

  // Know 0 <= B < L - V. Take in B = L - V - b as initial guess
  int operator()(const Eigen::VectorXd & B, Eigen::VectorXd & fvec) const
  {
    if (B.size() < 1)
    {
      ignerr << "Invalid input size for CatenaryHSoln::operator()" << std::endl;
      return -1;
    }

    // Evaluate target function
    fvec[0] = this->InverseCatenaryVSoln(B[0]) - this->H;

    return 0;
  }
};

#endif  // MOORINGFORCE__CATENARYSOLN_HPP_
