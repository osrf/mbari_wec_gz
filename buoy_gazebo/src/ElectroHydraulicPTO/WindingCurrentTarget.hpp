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

#ifndef ELECTROHYDRAULICPTO__WINDINGCURRENTTARGET_HPP_
#define ELECTROHYDRAULICPTO__WINDINGCURRENTTARGET_HPP_

#include <splinter_ros/splinter1d.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>


// Defines from Controller Firmware, behavior replicated here
#define TORQUE_CONSTANT 0.438   // 0.62 N-m/ARMS  0.428N-m/AMPS Flux Current
#define CURRENT_CMD_RATELIMIT 200  // A/second.  Set to zero to disable feature
#define TORQUE_CMD_TIMEOUT 2  // Torque Command Timeut, in secs. Set to zero to disable timeout
#define BIAS_CMD_TIMEOUT 10  // Bias Current Command Timeut, secs. Set to zero to disable timeout
#define DEFAULT_SCALE_FACTOR 1.0  // -RPM on Kollemogen is +RPM here and extension
#define MAX_SCALE_FACTOR 1.4
#define MIN_SCALE_FACTOR 0.5
#define DEFAULT_RETRACT_FACTOR 0.6
#define MAX_RETRACT_FACTOR 1.0
#define MIN_RETRACT_FACTOR 0.4
#define DEFAULT_BIASCURRENT 0.0  // Start with zero bias current
#define MAX_BIASCURRENT 20.0  // Max allowable winding bias current Magnitude that can be applied
#define MAX_WINDCURRENTLIMIT 35  // Winding Current Limit, Amps.  Limit on internal target
#define SC_RANGE_MIN 0.0  // Inches
#define SC_RANGE_MAX 80.0  // Inches
#define STOP_RANGE 10.0  // Inches from SC_RANGE_MIN and SC_RANGE_MAX to increase generator torque
// Max amount to modify RPM in determining WindingCurrentLimit near ends of stroke
#define MAX_RPM_ADJUSTMENT 5000.0


class WindingCurrentTarget
{
public:
  const std::vector<double> NSpec {0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0};  // RPM
  const std::vector<double> TorqueSpec {0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6};  // N-m

  double TorqueConstantNMPerAmp;  // N-m/Amp
  double TorqueConstantInLbPerAmp;  // in-lb/Amp

  double RamPosition;
  double ScaleFactor;
  double RetractFactor;
  double UserCommandedCurrent{0.0};
  double BiasCurrent;
  mutable double I{0.0};
  mutable double J_I{0.0};
  bool current_override_{false};
  bool bias_override_{false};

  splinter_ros::Splinter1d DefaultDamping;

public:
  WindingCurrentTarget()
  : DefaultDamping(NSpec, TorqueSpec)
  {
    // Set Electric Motor Torque Constant
    this->TorqueConstantNMPerAmp = TORQUE_CONSTANT;  // N-m/Amp
    this->TorqueConstantInLbPerAmp = this->TorqueConstantNMPerAmp * 8.851;  // in-lb/Amp

    this->ScaleFactor = DEFAULT_SCALE_FACTOR;
    this->RetractFactor = DEFAULT_RETRACT_FACTOR;
    this->BiasCurrent = DEFAULT_BIASCURRENT;

    this->RamPosition = 0;  // Default to full retract, should be set before () operator is used.
  }

  double df(const double & N) const
  {
    if (current_override_) {
      J_I = 0.0;
    } else {
      J_I = this->DefaultDamping.evalJacobian(fabs(N), splinter_ros::USE_BOUNDS) *
        this->ScaleFactor / this->TorqueConstantNMPerAmp;

      if (N > 0.0) {
        J_I *= -this->RetractFactor;
      }
    }

    return J_I;
  }

  double operator()(const double & N) const
  {
    if (current_override_) {
      I = UserCommandedCurrent;
      // std::cerr << "User Commanded Current: [" << I << "]" << std::endl;
    } else {
      I = this->DefaultDamping.eval(
        fabs(N),
        splinter_ros::FILL_VALUE,
        std::vector<double>{
        TorqueSpec.front(),
        TorqueSpec.back()}) *
        this->ScaleFactor / this->TorqueConstantNMPerAmp;

      if (N > 0.0) {
        I *= -this->RetractFactor;
      }

      if (bias_override_) {
        I += BiasCurrent;
      }
    }

// Enforce Min/Max
//  - The winding current target is always constrained between +/- MAX_WINDCURRENTLIMIT
//  - At high speeds, the winding current is further limited to force the system into
//    a generating quadrant, eventually creating maximum resistance at or above 6000RPM.
//    This maximum resistance forces the pressure relief valve open in the system, which
//    reduces flow through the motor and keeps the speed from increasing further.
//  - Except when in "permissive mode", the RPM at which the system is forced into a
//    generating quadrant reduces as the piston nears the end.  This is effectively
//    a soft stop that prevents a commanded current from slamming the piston into the stop.
//
//   -6000RPM                               ^  I                   5000RPM
//       |                                  |                        |
//       V                                  |                        V
//    --------------------------------------|-------------------------  <- +35A
//        \                                 |                         \
//         \                                |                          \
//          \            (generating)       |        (motoring)         \
//           \                              |                            \
//            \                             |                             \
//             \                            |                              \
// <---------------------------------------------------------------------------------------->
//               \                          |                                \            RPM
//                \                         |                                 \
//                 \                        |                                  \
//                  \     (motoring)        |        (generating)               \
//                   \                      |                                    \
//                    \                     |                                     \
//            -35A ->  ---------------------|------------------------------------------
//                     ^                    |                                      ^
//                     |                    |                                      |
//                 -5000RPM                 V                                   6000RPM
    double AdjustedN = N;
    if (N >= 0.0) {  // Retracting
      if (RamPosition < (STOP_RANGE - SC_RANGE_MIN)) {
        AdjustedN += ((STOP_RANGE - SC_RANGE_MIN) - RamPosition) * MAX_RPM_ADJUSTMENT;
      }
      double CurrLim = -AdjustedN * 2.0 * MAX_WINDCURRENTLIMIT / 1000.0 + 385.0;  // Magic nums
      if (I > CurrLim) {
        I = CurrLim;
      }
    } else {  // Extending
      if (RamPosition > (SC_RANGE_MAX - STOP_RANGE)) {
        AdjustedN -= (RamPosition - (SC_RANGE_MAX - STOP_RANGE)) * MAX_RPM_ADJUSTMENT;
      }
      double CurrLim = -AdjustedN * 2.0 * MAX_WINDCURRENTLIMIT / 1000.0 - 385.0;  //  Magic nums
      if (I < CurrLim) {
        I = CurrLim;
      }
    }

    if (I < -MAX_WINDCURRENTLIMIT) {
      I = -MAX_WINDCURRENTLIMIT;
    }
    if (I > MAX_WINDCURRENTLIMIT) {
      I = MAX_WINDCURRENTLIMIT;
    }
    return I;
  }
};

#endif  // ELECTROHYDRAULICPTO__WINDINGCURRENTTARGET_HPP_
