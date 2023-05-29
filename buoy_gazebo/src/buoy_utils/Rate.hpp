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

#ifndef BUOY_UTILS__RATE_HPP_
#define BUOY_UTILS__RATE_HPP_


#include <chrono>

#include <rclcpp/macros.hpp>
#include <rclcpp/rate.hpp>


namespace buoy_utils
{

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

class SimRate : public rclcpp::RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SimRate);

  explicit SimRate(const double & rate, rclcpp::Clock::SharedPtr _clock)
  : SimRate(
      duration_cast<nanoseconds>(duration<double>(1.0 / rate)),
      _clock
  )
  {}

  explicit SimRate(
    const std::chrono::nanoseconds & period,
    rclcpp::Clock::SharedPtr _clock)
  : clock(_clock),
    period_(period),
    last_interval_(
      rclcpp::Time(_clock->now(),
      _clock->get_clock_type())
    )
  {}

  virtual bool sleep()
  {
    // Time coming into sleep
    auto now = clock->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (time_to_sleep <= rclcpp::Duration(0, 0U)) {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    clock->sleep_for(time_to_sleep);
    return true;
  }

  virtual bool is_steady() const
  {
    return false;
  }

  virtual void reset()
  {
    last_interval_ = clock->now();
  }

  rclcpp::Duration period() const
  {
    return period_;
  }

private:
  RCLCPP_DISABLE_COPY(SimRate)

  rclcpp::Clock::SharedPtr clock = nullptr;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
};

}  // namespace buoy_utils

#endif  // BUOY_UTILS__RATE_HPP_
