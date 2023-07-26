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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

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

  diagnostic_msgs::msg::DiagnosticStatus to_diagnostic_status(
    const std::string & name, const std::string & message,
    const T & warn_threshold)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;

    auto status = DiagnosticStatus();
    KeyValue & max = status.values.emplace_back();
    max.key = "max";
    max.value = std::to_string(this->max);
    KeyValue & mean = status.values.emplace_back();
    mean.key = "mean";
    mean.value = std::to_string(this->mean);
    KeyValue & min = status.values.emplace_back();
    min.key = "min";
    min.value = std::to_string(this->min);

    if (this->max > warn_threshold) {
      status.level = DiagnosticStatus::WARN;
    } else {
      status.level = DiagnosticStatus::OK;
    }
    status.name = name;
    status.message = message;
    return status;
  }
};

template<typename T = std::chrono::duration<double, std::milli>>
class CycleProfiler
{
public:
  typedef T Duration;
  typedef std::shared_ptr<CycleProfiler> SharedPtr;
  typedef std::unique_ptr<CycleProfiler> UniquePtr;

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

  void add_cycle_stats(const Duration & duration)
  {
    std::lock_guard<std::mutex> lock(mutex_);
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
