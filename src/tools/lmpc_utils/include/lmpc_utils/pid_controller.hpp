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


#ifndef LMPC_UTILS__PID_CONTROLLER_HPP_
#define LMPC_UTILS__PID_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace lmpc
{
namespace utils
{
struct PidCoefficients
{
  double k_p {};
  double k_i {};
  double k_d {};
  double min_cmd {};
  double max_cmd {};
  double min_i {};
  double max_i {};
};

class PidController
{
public:
  explicit PidController(std::string const & name, PidCoefficients const & coefficients);

  PidController();

  bool try_update_param(std::string const & name, rclcpp::Parameter const & param);

  void reset_integral_error(double integral_error);

  double integral_error();

  double update(double new_error, double actual_dt);

  const PidCoefficients & params() const;

private:
  std::string name_;
  PidCoefficients coefficients_;

  double integral_error_ {};
  double last_error_ {};
  double error_ {};
};
}  // namespace utils
}  // namespace lmpc
#endif  // LMPC_UTILS__PID_CONTROLLER_HPP_
