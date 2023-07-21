// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef LMPC_UTILS__CYCLE_PROFILER_HPP_
#define LMPC_UTILS__CYCLE_PROFILER_HPP_

#include <chrono>
#include <mutex>
#include <memory>
#include <string>
#include <algorithm>

#include <boost/circular_buffer.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace lmpc
{
namespace utils
{
template<typename T = std::chrono::duration<double, std::milli>>
struct Profile
{
  T mean;
  T max;
  T min;

  diagnostic_msgs::msg::DiagnosticArray to_diagnostic_status(
    const std::string & name,
    const T & warn_threshold)
  {
    using diagnostic_msgs::msg::DiagnosticArray;
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;

    DiagnosticArray diagnostics;
    DiagnosticStatus & status = diagnostics.status.emplace_back();
    KeyValue & max = status.values.emplace_back();
    max.key = "max (ms)";
    max.value = std::to_string(this->max.count());
    KeyValue & mean = status.values.emplace_back();
    mean.key = "mean (ms)";
    mean.value = std::to_string(this->mean.count());
    KeyValue & min = status.values.emplace_back();
    min.key = "min (ms)";
    min.value = std::to_string(this->min.count());

    if (this->max > warn_threshold) {
      status.level = DiagnosticStatus::WARN;
    } else {
      status.level = DiagnosticStatus::OK;
    }
    status.name = name;
    status.message = "Cycle Profile";
    return diagnostics;
  }
};

template<typename T = std::chrono::duration<double, std::milli>>
class CycleProfiler
{
public:
  typedef T Duration;
  typedef std::shared_ptr<CycleProfiler> SharedPtr;

  CycleProfiler()
  {
  }

  explicit CycleProfiler(const size_t & window)
  : durations_(window)
  {
  }

  void set_window(const size_t & window)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    durations_.resize(window);
  }

  void start()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_start_time_ = now();
  }

  void end()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Duration duration = std::chrono::duration_cast<Duration>(now() - last_start_time_);
    durations_.push_back(duration);
  }

  size_t capacity()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return durations_.capacity();
  }

  Profile<Duration> profile()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Profile<Duration> result{Duration(0), Duration(0), Duration(0)};
    const auto size = durations_.size();
    if (size == 0) {
      return result;
    } else {
      result.max = durations_[0];
      result.min = durations_[0];
    }
    for (const auto & duration : durations_) {
      result.max = std::max(result.max, duration);
      result.min = std::min(result.min, duration);
      result.mean += duration;
    }
    result.mean /= size;
    return result;
  }

protected:
  typedef boost::circular_buffer<Duration> DurationBuffer;
  typedef std::chrono::system_clock::time_point TimePoint;

  DurationBuffer durations_;
  TimePoint last_start_time_;
  std::mutex mutex_;

  TimePoint now()
  {
    return std::chrono::system_clock::now();
  }
};
}  // namespace utils
}  // namespace lmpc
#endif  // LMPC_UTILS__CYCLE_PROFILER_HPP_
