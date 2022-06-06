// Copyright 2022 Open Source Robotics Foundation
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

#ifndef BUOY_UTILS__STOPWATCHSIMTIME_HPP_
#define BUOY_UTILS__STOPWATCHSIMTIME_HPP_

#include <rclcpp/clock.hpp>

#include <chrono>
#include <memory>


namespace buoy_utils
{
// Forward declarations.
class StopwatchSimTimePrivate;

//
/// \class StopwatchSimTime StopwatchSimTime.hh buoy_utils/StopwatchSimTime.hh
/// \brief The stopwatch keeps track of time spent in the run state,
/// accessed through ElapsedRunTime(), and time spent in the stop state,
/// accessed through ElapsedStopTime(). Elapsed run time starts accumulating
/// after the first call to Start(). Elapsed stop time starts
/// accumulation after Start() has been called followed by Stop(). The
/// stopwatch can be reset with the Reset() function.
///
/// # Example usage
///
/// ```{.cpp}
/// buoy_utils::StopwatchSimTime watch;
/// watch.Start();
///
/// // do something...
///
/// std::cout << "Elapsed time is "
/// << std::chrono::duration_cast<std::chrono::milliseconds>(
///   timeSys.ElapsedRunTime()).count() << " ms\n";
/// watch.Stop();
/// ```
class StopwatchSimTime
{
  /// \brief Constructor.

public:
  StopwatchSimTime();

  /// \brief Copy constructor
  /// \param[in] _watch The stop watch to copy.

public:
  StopwatchSimTime(const StopwatchSimTime & _watch);

  /// \brief Move constructor
  /// \param[in] _watch The stop watch to move.

public:
  StopwatchSimTime(StopwatchSimTime && _watch) noexcept;

  /// \brief Destructor.

public:
  virtual ~StopwatchSimTime();

public:
  /// \brief Take a clock instance (e.g. get_clock() from rclcpp::Node).
  /// Can also follow sim time on /clock when node's use_sim_time param is set
  void SetClock(rclcpp::Clock::SharedPtr _clock);

  /// \brief Start the stopwatch.
  /// \param[in] _reset If true the stopwatch is reset first.
  /// \return True if the the stopwatch was started. This will return
  /// false if the stopwatch was already running.

public:
  bool Start(const bool _reset = false);

  /// \brief Get the time when the stopwatch was started.
  /// \return The time when stopwatch was started, or
  /// std::chrono::steady_clock::time_point::min() if the stopwatch
  /// has not been started.

public:
  const rclcpp::Time & StartTime() const;

  /// \brief Stop the stopwatch
  /// \return True if the stopwatch was stopped. This will return false
  /// if the stopwatch is not running.

public:
  bool Stop();

  /// \brief Get the time when the stopwatch was last stopped.
  /// \return The time when stopwatch was last stopped, or
  /// std::chrono::steady_clock::time_point::min() if the stopwatch
  /// has never been stopped.

public:
  const rclcpp::Time & StopTime() const;

  /// \brief Get whether the stopwatch is running.
  /// \return True if the stopwatch is running.

public:
  bool Running() const;

  /// \brief Reset the stopwatch. This resets the start time, stop time,
  /// elapsed duration and elapsed stop duration.

public:
  void Reset();

  /// \brief Get the amount of time that the stop watch has been
  /// running. This is the total amount of run time, spannning all start
  /// and stop calls. The Reset function or passing true to the Start
  /// function will reset this value.
  /// \return Total amount of elapsed run time.

public:
  rclcpp::Duration ElapsedRunTime() const;

  /// \brief Get the amount of time that the stop watch has been
  /// stopped. This is the total amount of stop time, spannning all start
  /// and stop calls. The Reset function or passing true to the Start
  /// function will reset this value.
  /// \return Total amount of elapsed stop time.

public:
  rclcpp::Duration ElapsedStopTime() const;

  /// \brief Equality operator.
  /// \param[in] _watch The watch to compare.
  /// \return True if this watch equals the provided watch.

public:
  bool operator==(const StopwatchSimTime & _watch) const;

  /// \brief Inequality operator.
  /// \param[in] _watch The watch to compare.
  /// \return True if this watch does not equal the provided watch.

public:
  bool operator!=(const StopwatchSimTime & _watch) const;

  /// \brief Copy assignment operator
  /// \param[in] _watch The stop watch to copy.
  /// \return Reference to this.

public:
  StopwatchSimTime & operator=(const StopwatchSimTime & _watch);

  /// \brief Move assignment operator
  /// \param[in] _watch The stop watch to move.
  /// \return Reference to this.

public:
  StopwatchSimTime & operator=(StopwatchSimTime && _watch);

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
  /// \brief Private data pointer.

private:
  std::unique_ptr<StopwatchSimTimePrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
};
}  // namespace buoy_utils
#endif  // BUOY_UTILS__STOPWATCHSIMTIME_HPP_
