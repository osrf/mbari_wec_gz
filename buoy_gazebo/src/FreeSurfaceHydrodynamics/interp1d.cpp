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

#include "interp1d.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>
#include <utility>


namespace simple_interp
{
Interp1d::Interp1d(const std::vector<double> & x, const std::vector<double> & y)
: table_{Interp1d::make_table(x, y)}
{
}

Interp1d::Interp1d(const table_t & table)
: table_{table}
{
}

void Interp1d::update(const std::vector<double> & x, const std::vector<double> & y)
{
  table_ = Interp1d::make_table(x, y);
}

Interp1d::table_t Interp1d::make_table(
  const std::vector<double> & x,
  const std::vector<double> & y)
{
  assert(x.size() == y.size() && "x and y should have the same size");
  table_t table; table.reserve(x.size());
  for (auto citx = std::make_pair(x.cbegin(), y.cbegin());
    citx.first != x.cend();
    ++citx.first, ++citx.second)
  {
    table.push_back(std::make_pair(*citx.first, *citx.second));
  }
  return table;
}

// static: one-shot eval with vector data
double Interp1d::eval(
  const std::vector<double> & x_table,
  const std::vector<double> & y_table,
  const double & x_eval)
{
  return Interp1d(x_table, y_table).eval(x_eval);
}

// static: one-shot eval with table data
double Interp1d::eval(const Interp1d::table_t & table, const double & x)
{
  return Interp1d(table).eval(x);
}

// operator version of eval with class instance
double Interp1d::operator()(const double & x) const
{
  return this->eval(x);
}

// eval with class instance
double Interp1d::eval(const double & x) const
{
  // use bounds of data table and short-cut
  if (x <= table_.front().first) {
    return table_.front().second;
  } else if (x >= table_.back().first) {
    return table_.back().second;
  }

  // check if x is in the previous bin +/- 1 and shortcut the search
  bool x_in_latest_bin{false};
  if (latest_bin_initialized_) {
    x_in_latest_bin =
      ((latest_upper_edge_ - 1U)->first < x) &&
      (x <= latest_upper_edge_->first);

    if (!x_in_latest_bin && latest_upper_edge_ + 1 != table_.end()) {
      // try up one bin
      x_in_latest_bin =
        (latest_upper_edge_->first < x) &&
        (x <= (latest_upper_edge_ + 1U)->first);
      latest_upper_edge_ = latest_upper_edge_ + 1U;
    }

    if (!x_in_latest_bin && latest_upper_edge_ - 1U != table_.begin()) {
      // now try down one bin
      x_in_latest_bin =
        ((latest_upper_edge_ - 2U)->first < x) &&
        (x <= (latest_upper_edge_ - 1U)->first);
      latest_upper_edge_ = latest_upper_edge_ - 1U;
    }
  }

  // search for correct bin otherwise shortcut with same bin
  if (!x_in_latest_bin) {
    const double dummy_y{0.0};
    table_t::const_iterator citx =
      std::lower_bound(
      table_.cbegin(),
      table_.cend(),
      point_t(x, dummy_y));

    if (citx == table_.end()) {
      assert(citx != table_.end() && "reached end of table without finding a bin");
      return 0.0;  // shouldn't get here with bounds checking
    }

    latest_upper_edge_ = citx;
    if (!latest_bin_initialized_) {
      latest_bin_initialized_ = true;
    }
  }

  // compute linear interpolation
  const double & x1 = latest_upper_edge_->first;
  const double & y1 = latest_upper_edge_->second;
  const double & x0 = (latest_upper_edge_ - 1U)->first;
  const double & y0 = (latest_upper_edge_ - 1U)->second;
  const double frac = (x - x0) / (x1 - x0);
  const double y = (y0 * (1.0 - frac)) + (y1 * frac);

  return y;
}
}  // namespace simple_interp
