// Copyright 2023 Haoru Xue
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

#ifndef LMPC_UTILS__LOGGING_HPP_
#define LMPC_UTILS__LOGGING_HPP_

#include <stdint.h>
#include <map>
#include <string>
#include "casadi/casadi.hpp"

namespace lmpc
{
namespace utils
{
enum LogLevel : uint8_t
{
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
  FATAL = 4
};

class Logger
{
public:
  typedef void (* LoggerCallback)(const LogLevel &, const std::string &);

protected:
  typedef void (* LoggerCallback)(const LogLevel &, const std::string &);
  typedef std::map<LoggerCallback, LogLevel> LoggerCallbackDict;

  LoggerCallbackDict callbacks_;

public:
  Logger() {}

  /**
   * @brief Register a new callback function for this logger.
   *
   * @param callback a callback function taking a LogLevel and a std::string message.
   * @param min_level minimum log level to trigger this callback. Default to debug.
   *
   * @note a callback can be registered only once. re-registration will overwrite the previous minimum log level.
   */
  void register_callback(
    const LoggerCallback & callback,
    const LogLevel & min_level = LogLevel::DEBUG)
  {
    if (callback) {
      callbacks_[callback] = min_level;
    }
  }

  /**
   * @brief Unregister a callback from the logger.
   *
   * @param callback the callback that was previously registered.
   *
   * @returns True if the callback exists and is unregistered.
   * @returns False if the callback was never registered.
   */
  bool unregister_callback(const LoggerCallback & callback)
  {
    return callbacks_.erase(callback);
  }

  /**
   * @brief Send a log to the callbacks.
   *
   * @param level log level.
   * @param what message.
   */
  void send_log(const LogLevel & level, const std::string & what)
  {
    for (const auto & callback : callbacks_) {
      if (level > callback.second) {
        callback.first(level, what);
      }
    }
  }
};

}  // namespace utils
}  // namespace lmpc

#endif  // LMPC_UTILS__LOGGING_HPP_
