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

#include "racing_mpc/ros_param_loader.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
RacingMPCConfig::SharedPtr load_parameters(rclcpp::Node * node)
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

  const auto step_mode_str = declare_string("racing_mpc.step_mode");
  RacingMPCStepMode step_mode;
  if (step_mode_str == "step") {
    step_mode = RacingMPCStepMode::STEP;
  } else if (step_mode_str == "continuous") {
    step_mode = RacingMPCStepMode::CONTINUOUS;
  } else {
    throw std::invalid_argument("Invalid step mode: " + step_mode_str);
  }

  const auto R = casadi::DM(declare_vec("racing_mpc.r"));
  const auto R_d = casadi::DM(declare_vec("racing_mpc.r_d"));

  return std::make_shared<RacingMPCConfig>(
    RacingMPCConfig{
          declare_double("racing_mpc.max_cpu_time"),
          declare_int("racing_mpc.max_iter"),
          declare_double("racing_mpc.tol"),
          static_cast<size_t>(declare_int("racing_mpc.n")),
          declare_double("racing_mpc.margin"),
          declare_double("racing_mpc.average_track_width"),
          declare_bool("racing_mpc.verbose"),
          declare_bool("racing_mpc.jit"),
          step_mode,
          casadi::DM(declare_double("racing_mpc.q_contour")),
          casadi::DM(declare_double("racing_mpc.q_heading")),
          casadi::DM(declare_double("racing_mpc.q_vel")),
          casadi::DM(declare_double("racing_mpc.q_boundary")),
          casadi::DM::reshape(
            R, static_cast<casadi_int>(sqrt(R.size1())),
            static_cast<casadi_int>(sqrt(R.size1()))),
          casadi::DM::reshape(
            R_d, static_cast<casadi_int>(sqrt(R_d.size1())),
            static_cast<casadi_int>(sqrt(R_d.size1()))),
          casadi::DM(declare_vec("racing_mpc.x_max")),
          casadi::DM(declare_vec("racing_mpc.x_min")),
          casadi::DM(declare_vec("racing_mpc.u_max")),
          casadi::DM(declare_vec("racing_mpc.u_min")),
        }
  );
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
