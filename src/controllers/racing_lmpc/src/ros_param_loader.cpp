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

#include <string>
#include <memory>
#include <vector>

#include <lmpc_utils/ros_param_helper.hpp>

#include "racing_lmpc/ros_param_loader.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_lmpc
{
RacingLMPCConfig::SharedPtr load_parameters(rclcpp::Node * node)
{
  auto declare_double = [&](const char * name) {
      return lmpc::utils::declare_parameter<double>(node, name);
    };
  auto declare_string = [&](const char * name) {
      return lmpc::utils::declare_parameter<std::string>(node, name);
    };
  auto declare_vec = [&](const char * name) {
      return lmpc::utils::declare_parameter<std::vector<double>>(node, name);
    };
  auto declare_int = [&](const char * name) {
      return lmpc::utils::declare_parameter<int64_t>(node, name);
    };
  auto declare_bool = [&](const char * name) {
      return lmpc::utils::declare_parameter<bool>(node, name);
    };

  const auto step_mode_str = declare_string("racing_lmpc.step_mode");
  RacingLMPCStepMode step_mode;
  if (step_mode_str == "step") {
    step_mode = RacingLMPCStepMode::STEP;
  } else if (step_mode_str == "continuous") {
    step_mode = RacingLMPCStepMode::CONTINUOUS;
  } else {
    throw std::invalid_argument("Invalid step mode: " + step_mode_str);
  }

  return std::make_shared<RacingLMPCConfig>(
    RacingLMPCConfig{
          declare_double("racing_lmpc.max_cpu_time"),
          declare_int("racing_lmpc.max_iter"),
          declare_double("racing_lmpc.tol"),
          static_cast<size_t>(declare_int("racing_lmpc.n")),
          declare_double("racing_lmpc.margin"),
          declare_double("racing_lmpc.average_track_width"),
          declare_bool("racing_lmpc.verbose"),
          step_mode,
          casadi::DM(declare_double("racing_lmpc.q_contour")),
          casadi::DM(declare_double("racing_lmpc.q_heading")),
          casadi::DM(declare_double("racing_lmpc.q_vel")),
          casadi::DM(declare_double("racing_lmpc.q_boundary")),
          casadi::DM::reshape(casadi::DM(declare_vec("racing_lmpc.r")), 3, 3),
          casadi::DM(declare_vec("racing_lmpc.x_max")),
          casadi::DM(declare_vec("racing_lmpc.x_min")),
          casadi::DM(declare_vec("racing_lmpc.u_max")),
          casadi::DM(declare_vec("racing_lmpc.u_min")),
        }
  );
}
}  // namespace racing_lmpc
}  // namespace mpc
}  // namespace lmpc
