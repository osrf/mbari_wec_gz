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

#include <stdio.h>

#include <JustInterp/JustInterp.hpp>

#include <cmath>
#include <iostream>
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
#define SC_RANGE_MIN 0  // Inches
#define SC_RANGE_MAX 80  // Inches
#define STOP_RANGE 10  // Inches from SC_RANGE_MIN and SC_RANGE_MAX to increase generator torque
// Max amount to modify RPM in determining WindingCurrentLimit near ends of stroke
#define MAX_RPM_ADJUSTMENT 5000


class WindingCurrentTarget
{
public:
  double TorqueConstantNMPerAmp;  // N-m/Amp
  double TorqueConstantInLbPerAmp;  // in-lb/Amp
  JustInterp::LinearInterpolator<double> DefaultDamping;
  double ScaleFactor;
  double RetractFactor;
  double UserCommandedCurrent{0.0};
  double BiasCurrent;
  double I{0.0};
  bool current_override_{false};
  bool bias_override_{false};

public:
  /// \brief mutex to protect jointVelCmd
  std::mutex UserCommandMutex;

  WindingCurrentTarget()
  {
    std::vector<double> N {0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0};  // RPM
    std::vector<double> Torque {0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6};  // N-m
    this->DefaultDamping.SetData(N.size(), N.data(), Torque.data());

    // Set Electric Motor Torque Constant
    this->TorqueConstantNMPerAmp = TORQUE_CONSTANT;  // N-m/Amp
    this->TorqueConstantInLbPerAmp = this->TorqueConstantNMPerAmp * 8.851;  // in-lb/Amp

    this->ScaleFactor = DEFAULT_SCALE_FACTOR;
    this->RetractFactor = DEFAULT_RETRACT_FACTOR;
    this->BiasCurrent = DEFAULT_BIASCURRENT;
  }

  double operator()(const double & N) const
  {
    double I;
    if (current_override_) {
      I = UserCommandedCurrent;
    } else {
      // TODO(anyone):  1.375 makes this match experiment, not sure what is wrong...
      I = this->DefaultDamping(fabs(N)) * this->ScaleFactor / this->TorqueConstantNMPerAmp;
      if (N > 0.0) {
        I *= -this->RetractFactor;
      }

      if (bias_override_) {
        std::cerr << "using override current" << std::endl;
        I += BiasCurrent;
      }
    }

    return I;
  }
};


#endif  // ELECTROHYDRAULICPTO__WINDINGCURRENTTARGET_HPP_
