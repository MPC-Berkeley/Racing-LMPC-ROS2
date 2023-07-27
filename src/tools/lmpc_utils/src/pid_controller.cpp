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


#include <cmath>
#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "lmpc_utils/pid_controller.hpp"

namespace lmpc
{
namespace utils
{
PidController::PidController(std::string const & name, PidCoefficients const & coefficients)
: name_{name}, coefficients_{coefficients}
{
}

PidController::PidController()
: name_{"uninitialized"}, coefficients_{{}}
{
}

bool PidController::try_update_param(std::string const & name, rclcpp::Parameter const & param)
{
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    RCLCPP_WARN(
      rclcpp::get_logger(name_), "%s is not double", name.c_str());
    return false;
  }

  const auto value = param.as_double();
  if (name == "kp") {
    coefficients_.k_p = value;
  } else if (name == "ki") {
    coefficients_.k_i = value;
  } else if (name == "kd") {
    coefficients_.k_d = value;
  } else if (name == "min_cmd") {
    coefficients_.min_cmd = value;
  } else if (name == "max_cmd") {
    coefficients_.max_cmd = value;
  } else if (name == "min_i") {
    coefficients_.min_i = value;
  } else if (name == "max_i") {
    coefficients_.max_i = value;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger(name_), "%s does not exist", name.c_str());
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(name_), "%s = %f", name.c_str(), value);
  return true;
}

void PidController::reset_integral_error(double integral_error)
{
  integral_error_ = integral_error;
}

double PidController::integral_error()
{
  return integral_error_;
}

double PidController::update(double new_error, double actual_dt)
{
  if (std::isnan(new_error)) {
    RCLCPP_WARN(rclcpp::get_logger(name_), "WARNING: error -> NAN");
    return NAN;
  }

  last_error_ = error_;
  error_ = new_error;

  // Calculate & saturate integral error.

  integral_error_ += error_ * actual_dt;
  integral_error_ = std::clamp(integral_error_, coefficients_.min_i, coefficients_.max_i);

  // Calculate dt error.

  const auto dt_error = (error_ - last_error_) / actual_dt;

  // Calculate control.

  const auto p = error_ * coefficients_.k_p;
  const auto i = integral_error_ * coefficients_.k_i;
  const auto d = dt_error * coefficients_.k_d;

  const auto cmd = p + i + d;

  // Clamp control.

  if (cmd <= coefficients_.min_cmd) {
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     name_), "WARNING: clamped %f to min %f", cmd, coefficients_.min_cmd);
    return coefficients_.min_cmd;
  }

  if (cmd >= coefficients_.max_cmd) {
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     name_), "WARNING: clamped %f to max %f", cmd, coefficients_.max_cmd);
    return coefficients_.max_cmd;
  }

  return cmd;
}

const PidCoefficients & PidController::params() const
{
  return coefficients_;
}
}  // namespace utils
}  // namespace lmpc
