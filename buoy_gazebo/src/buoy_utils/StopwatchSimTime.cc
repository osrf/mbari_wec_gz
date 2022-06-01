/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "StopwatchSimTime.hh"

#include <chrono>
#include <limits>

#include <rclcpp/clock.hpp>


using namespace buoy_utils;

const rclcpp::Duration duration_zero = rclcpp::Duration(0, 0U);
const rclcpp::Time time_min = rclcpp::Time(0, 0U);

// Private data class
class buoy_utils::StopwatchSimTimePrivate
{
  /// \brief Default constructor.
  public: StopwatchSimTimePrivate() = default;

  /// \brief Copy constructor.
  /// \param[in] _watch Watch to copy.
  public: explicit StopwatchSimTimePrivate(const StopwatchSimTimePrivate &_watch)
          : running(_watch.running),
            startTime(_watch.startTime),
            stopTime(_watch.stopTime),
            stopDuration(_watch.stopDuration),
            runDuration(_watch.runDuration)
  {
  }

  /// \brief True if the real time clock is running.
  public: bool running = false;

  /// \brief Time point that marks the start of the real-time clock.
  public: rclcpp::Time startTime = time_min;

  /// \brief Time point that marks the stop of the real-time clock.
  public: rclcpp::Time stopTime = time_min;

  /// \brief Amount of stop time.
  public: rclcpp::Duration stopDuration = duration_zero;

  /// \brief Amount of run time.
  public: rclcpp::Duration runDuration = duration_zero;
  
  /// \brief ros clock instance
  public: rclcpp::Clock::SharedPtr clock = nullptr;
};

//////////////////////////////////////////////////
StopwatchSimTime::StopwatchSimTime()
  : dataPtr(new StopwatchSimTimePrivate)
{
}

//////////////////////////////////////////////////
StopwatchSimTime::StopwatchSimTime(const StopwatchSimTime &_watch)
  : dataPtr(new StopwatchSimTimePrivate(*_watch.dataPtr))
{
}

//////////////////////////////////////////////////
StopwatchSimTime::StopwatchSimTime(StopwatchSimTime &&_watch) noexcept
  : dataPtr(std::move(_watch.dataPtr))
{
}

//////////////////////////////////////////////////
StopwatchSimTime::~StopwatchSimTime()
{
}

//////////////////////////////////////////////////
void StopwatchSimTime::SetClock(rclcpp::Clock::SharedPtr _clock)
{
  this->dataPtr->clock = _clock;
}

//////////////////////////////////////////////////
bool StopwatchSimTime::Start(const bool _reset)
{
  if (!this->dataPtr->clock)
    return false;

  if (_reset)
    this->Reset();

  if (!this->dataPtr->running)
  {
    if (this->dataPtr->startTime != this->dataPtr->stopTime)
    {
      this->dataPtr->stopDuration = this->dataPtr->stopDuration + \
        this->dataPtr->clock->now() - this->dataPtr->stopTime;
    }

    this->dataPtr->running = true;
    this->dataPtr->startTime = this->dataPtr->clock->now();
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
rclcpp::Time StopwatchSimTime::StartTime() const
{
  return this->dataPtr->startTime;
}

//////////////////////////////////////////////////
bool StopwatchSimTime::Stop()
{
  if (!this->dataPtr->clock)
    return false;

  if (this->dataPtr->running)
  {
    this->dataPtr->running = false;
    this->dataPtr->stopTime = this->dataPtr->clock->now();
    this->dataPtr->runDuration = this->dataPtr->runDuration + \
      this->dataPtr->stopTime - this->dataPtr->startTime;
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
rclcpp::Time StopwatchSimTime::StopTime() const
{
  return this->dataPtr->stopTime;
}

//////////////////////////////////////////////////
bool StopwatchSimTime::Running() const
{
  return this->dataPtr->running;
}

//////////////////////////////////////////////////
void StopwatchSimTime::Reset()
{
  this->dataPtr->running = false;
  this->dataPtr->startTime = time_min;
  this->dataPtr->stopTime = time_min;
  this->dataPtr->stopDuration = duration_zero;
  this->dataPtr->runDuration = duration_zero;
}

//////////////////////////////////////////////////
rclcpp::Duration StopwatchSimTime::ElapsedRunTime() const
{
  if (!this->dataPtr->clock)
    return duration_zero;

  if (this->dataPtr->running)
  {
    return this->dataPtr->clock->now() - this->dataPtr->startTime + this->dataPtr->runDuration;
  }
  else
  {
    return this->dataPtr->runDuration;
  }
}

//////////////////////////////////////////////////
rclcpp::Duration StopwatchSimTime::ElapsedStopTime() const
{
  if (!this->dataPtr->clock)
    return duration_zero;

  // If running, then return the stopDuration.
  if (this->dataPtr->running)
  {
    return this->dataPtr->stopDuration;
  }
  // The clock is not running, and Stop() has been called.
  else if (this->dataPtr->stopTime > time_min)
  {
    return this->dataPtr->stopDuration +
      (this->dataPtr->clock->now() - this->dataPtr->stopTime);
  }

  // Otherwise, the stopwatch has been reset or never started.
  return duration_zero;
}

//////////////////////////////////////////////////
bool StopwatchSimTime::operator==(const StopwatchSimTime &_watch) const
{
  return this->dataPtr->running == _watch.dataPtr->running &&
    this->dataPtr->startTime == _watch.dataPtr->startTime &&
    this->dataPtr->stopTime == _watch.dataPtr->stopTime &&
    this->dataPtr->stopDuration == _watch.dataPtr->stopDuration &&
    this->dataPtr->runDuration == _watch.dataPtr->runDuration;
}

//////////////////////////////////////////////////
bool StopwatchSimTime::operator!=(const StopwatchSimTime &_watch) const
{
  return !(*this == _watch);
}

//////////////////////////////////////////////////
StopwatchSimTime &StopwatchSimTime::operator=(const StopwatchSimTime &_watch)
{
  this->dataPtr.reset(new StopwatchSimTimePrivate(*_watch.dataPtr));
  return *this;
}

//////////////////////////////////////////////////
StopwatchSimTime &StopwatchSimTime::operator=(StopwatchSimTime &&_watch)
{
  this->dataPtr = std::move(_watch.dataPtr);
  return *this;
}
