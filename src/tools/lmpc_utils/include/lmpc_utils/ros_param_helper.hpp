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

#ifndef LMPC_UTILS__ROS_PARAM_HELPER_HPP_
#define LMPC_UTILS__ROS_PARAM_HELPER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace lmpc
{
namespace utils
{
template<typename T>
T declare_parameter(rclcpp::Node * node, const char * name)
{
  try {
    return node->declare_parameter<T>(name);
  } catch (rclcpp::exceptions::InvalidParameterValueException & e) {
    RCLCPP_FATAL(
      node->get_logger(), "Parameter \"%s\" value does not exsist or is invalid: %s", name,
      e.what());
    throw e;
  } catch (rclcpp::exceptions::InvalidParametersException & e) {
    RCLCPP_FATAL(
      node->get_logger(), "Parameter \"%s\" name is invalid: %s", name,
      e.what());
    throw e;
  } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_FATAL(
      node->get_logger(), "Parameter \"%s\" type mismatch: %s", name,
      e.what());
    throw e;
  } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_FATAL(
      node->get_logger(), "Parameter \"%s\" is already declared: %s", name,
      e.what());
    throw e;
  }
}
}  // namespace utils
}  // namespace lmpc
#endif  // LMPC_UTILS__ROS_PARAM_HELPER_HPP_
