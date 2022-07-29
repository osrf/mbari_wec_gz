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

#ifndef BUOY_UTILS__STATUS_HPP_
#define BUOY_UTILS__STATUS_HPP_


namespace buoy_utils
{
template<typename T>
union StatusUnion {
  uint16_t status{0U};
  T bits;
};

template<typename T>
struct Status
{
  StatusUnion<T> status;

  operator const uint16_t &() const
  {
    return status.status;
  }

  T & bits()
  {
    return status.bits;
  }
};

}  // namespace buoy_utils

#endif  // BUOY_UTILS__STATUS_HPP_
