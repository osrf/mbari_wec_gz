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

namespace buoy_gazebo
{

/////////////////////////////////////////////////
/// \brief Generic functor for solver.
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>
    JacobianType;

  const int m_inputs;
  const int m_values;

  /// \brief Constructor
  Functor()
  : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
  {
  }

  /// \brief Constructor
  Functor(int _inputs, int _values)
    : m_inputs(_inputs), m_values(_values)
  {
  }

  /// \brief The number of inputs.
  int inputs() const
  {
    return m_inputs;
  }

  /// \brief The number of outputs.
  int values() const
  {
    return m_values;
  }

  // you should define that in the subclass :
  // int operator()(const InputType &_x, ValueType *_v, JacobianType *_j = 0)
  // const;
};

/////////////////////////////////////////////////
struct CatenaryFunction
{
  /// \brief Scaling factor for catenary chain.
  /// \param _V: Vertical distance from buoy to anchor (meters).
  /// \param _B: Length of mooring chain lying on seafloor,
  /// start of catenary (metres).
  /// \param _L: Total length of mooring chain (metres).
  /// \return c, scaling factor, ratio of horizontal component of chain tension
  /// and weight of cable per unit length (N/m).
  public: static double CatenaryScalingFactor(double _V, double _B, double _L)
  {
    // Scaling factor c
    return (pow((_L - _B), 2) - pow(_V, 2)) / (2.0 * _V);
  }
};

/////////////////////////////////////////////////
struct CatenaryHSoln : Functor<double>
{
   /// \brief Vertical distance from buoy to anchor (meters).
   private: double V{std::nanf("")};

   /// \brief Horizontal distance from buoy to anchor (meters).
   private: double H{std::nanf("")};

   /// \brief Total length of mooring chain (metres).
   private: double L{std::nanf("")};

  /// \brief Constructor.
  public: CatenaryHSoln(double _V, double _H, double _L)
    : Functor<double>(1, 1),
    V(_V),
    H(_H),
    L(_L)
  {
  }

  /// \brief Invert the catenary equation and solve for the horizontal input.
  ///
  /// \param _B length of chain on floor.
  double InverseCatenaryVSoln(double _B) const
  {
    double c = CatenaryFunction::CatenaryScalingFactor(this->V, _B, this->L);
    return c * std::acosh(this->V / c + 1.0) + _B;
  }

  // Know 0 <= B < L - V. Take in B = L - V - b as initial guess
  int operator()(const Eigen::VectorXd &_B, Eigen::VectorXd &_fvec) const
  {
    if (_B.size() < 1) {
      gzerr << "Invalid input size for CatenaryHSoln::operator()" << std::endl;
      return -1;
    }

    // Evaluate target function
    _fvec[0] = this->InverseCatenaryVSoln(_B[0]) - this->H;

    return 0;
  }
};

}  // namespace buoy_gazebo

#endif  // MOORINGFORCE__CATENARYSOLN_HPP_
