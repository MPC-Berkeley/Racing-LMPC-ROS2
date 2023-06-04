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

#include "racing_lqr/ros_param_loader.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_lqr
{
RacingLQRConfig::SharedPtr load_parameters(rclcpp::Node * node)
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

  return std::make_shared<RacingLQRConfig>(
    RacingLQRConfig{
          static_cast<size_t>(declare_int("racing_lqr.n")),
          declare_double("racing_lqr.dt"),
          casadi::DM::reshape(casadi::DM(declare_vec("racing_lqr.q")), 6, 6),
          casadi::DM::reshape(casadi::DM(declare_vec("racing_lqr.r")), 3, 3),
          casadi::DM::reshape(casadi::DM(declare_vec("racing_lqr.qf")), 6, 6)
        }
  );
}
}  // namespace racing_lqr
}  // namespace mpc
}  // namespace lmpc
