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

#ifndef BUOY_UTILS__CONSTANTS_HPP_
#define BUOY_UTILS__CONSTANTS_HPP_

#include <cmath>

namespace buoy_utils
{
// Universal Constants
static constexpr double CubicInchesPerGallon{231.0};
static constexpr double SecondsPerMinute{60.0};
static constexpr double RPM_TO_RAD_PER_SEC{2.0 * M_PI / 60.0};
static constexpr double NM_PER_INLB{0.112984829};
static constexpr double INLB_PER_NM{8.851};
static constexpr double INCHES_PER_METER{39.4};
}  // namespace buoy_utils


#endif  // BUOY_UTILS__CONSTANTS_HPP_
