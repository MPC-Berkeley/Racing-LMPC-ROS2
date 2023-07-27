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

#ifndef VANILLA_CONTROLLER__VANILLA_CONTROLLER_CONFIG_HPP_
#define VANILLA_CONTROLLER__VANILLA_CONTROLLER_CONFIG_HPP_

#include <memory>
#include <vector>

#include <casadi/casadi.hpp>

#include <lmpc_utils/pid_controller.hpp>

namespace lmpc
{
namespace mpc
{
namespace vanilla_controller
{
enum VanillaControllerStepMode
{
  STEP,
  CONTINUOUS
};

struct VanillaControllerConfig
{
  typedef std::shared_ptr<VanillaControllerConfig> SharedPtr;

  // pure pursuit controller parameters
  double lookahead_speed_ratio = 1.0;
  double min_lookahead_distance = 1.0;
  double max_lookahead_distance = 10.0;

  // pid controller parameters
  utils::PidCoefficients lon_pid_coeffs;
  double dt = 0.1;

  // step mode
  VanillaControllerStepMode step_mode = VanillaControllerStepMode::STEP;
};
}  // namespace vanilla_controller
}  // namespace mpc
}  // namespace lmpc
#endif  // VANILLA_CONTROLLER__VANILLA_CONTROLLER_CONFIG_HPP_
