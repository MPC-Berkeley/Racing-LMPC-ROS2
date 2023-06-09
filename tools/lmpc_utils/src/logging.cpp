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

#include "lmpc_utils/logging.hpp"

namespace lmpc
{
namespace utils
{
Logger::Logger() {}

void Logger::register_callback(
  const LoggerCallbackKey & name,
  const LoggerCallback & callback,
  const LogLevel & min_level)
{
  if (callback) {
    callbacks_[name] = {callback, min_level};
  }
}

bool Logger::unregister_callback(const LoggerCallbackKey & name)
{
  return callbacks_.erase(name);
}

void Logger::send_log(const LogLevel & level, const std::string & what)
{
  for (const auto & callback : callbacks_) {
    if (level > callback.second.second) {
      callback.second.first(level, what);
    }
  }
}

Logger::LoggerCallback Logger::log_to_rclcpp(rclcpp::Node * node)
{
  return [node](const LogLevel & level, const std::string & what)
         {
           switch (level) {
             case LogLevel::DEBUG:
               RCLCPP_DEBUG(node->get_logger(), what.c_str());
               break;

             case LogLevel::INFO:
               RCLCPP_INFO(node->get_logger(), what.c_str());
               break;

             case LogLevel::WARN:
               RCLCPP_WARN(node->get_logger(), what.c_str());
               break;

             case LogLevel::ERROR:
               RCLCPP_ERROR(node->get_logger(), what.c_str());
               break;

             case LogLevel::FATAL:
               RCLCPP_FATAL(node->get_logger(), what.c_str());
               break;
           }
         };
}
}  // namespace utils
}  // namespace lmpc
