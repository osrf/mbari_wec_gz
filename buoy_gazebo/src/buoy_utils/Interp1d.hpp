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

#ifndef BUOY_UTILS__INTERP1D_HPP_
#define BUOY_UTILS__INTERP1D_HPP_


#include <vector>
#include <utility>


namespace buoy_utils
{
class Interp1d
{
public:
  //                  x       y
  typedef std::pair<double, double> point_t;
  typedef std::vector<point_t> table_t;

  Interp1d(const std::vector<double> & x, const std::vector<double> & y);
  explicit Interp1d(const table_t & table);

  void update(const std::vector<double> & x, const std::vector<double> & y);

  static table_t make_table(const std::vector<double> & x, const std::vector<double> & y);
  static double eval(
    const std::vector<double> & x_table,
    const std::vector<double> & y_table,
    const double & x_eval);
  static double eval(const table_t & table, const double & x);
  double eval(const double & x) const;

private:
  table_t table_;
  mutable table_t::const_iterator latest_upper_edge_;
  mutable bool latest_bin_initialized_{false};
};

}  // namespace buoy_utils

#endif  // BUOY_UTILS__INTERP1D_HPP_
