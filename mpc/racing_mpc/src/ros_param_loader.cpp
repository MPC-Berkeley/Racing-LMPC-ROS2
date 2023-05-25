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
  auto declare_vec = [&](const char * name) {
      return lmpc::utils::declare_parameter<std::vector<double>>(node, name);
    };
  auto declare_int = [&](const char * name) {
      return lmpc::utils::declare_parameter<int64_t>(node, name);
    };

  return std::make_shared<RacingMPCConfig>(
    RacingMPCConfig{
          declare_double("max_wall_time"),
          declare_double("tol"),
          declare_double("constr_viol_tol"),
          static_cast<size_t>(declare_int("n")),
          declare_double("margin"),
          declare_double("average_track_width"),
          casadi::DM(declare_vec("q")),
          casadi::DM(declare_vec("r")),
          casadi::DM(declare_vec("qf")),
          casadi::DM(declare_vec("x_max")),
          casadi::DM(declare_vec("x_min")),
          casadi::DM(declare_vec("u_max")),
          casadi::DM(declare_vec("u_min")),
        }
  );
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
