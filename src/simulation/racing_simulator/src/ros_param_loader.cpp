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

#include "racing_simulator/ros_param_loader.hpp"

namespace lmpc
{
namespace simulation
{
namespace racing_simulator
{
RacingSimulatorConfig::SharedPtr load_parameters(rclcpp::Node * node)
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
  // auto declare_int = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<int64_t>(node, name);
  //   };
  auto declare_bool = [&](const char * name) {
      return lmpc::utils::declare_parameter<bool>(node, name);
    };

  return std::make_shared<RacingSimulatorConfig>(
    RacingSimulatorConfig{
          declare_double("racing_simulator.dt"),
          declare_double("racing_simulator.repeat_state_dt"),
          declare_bool("racing_simulator.publish_tf"),
          declare_bool("racing_simulator.visualize_boundary"),
          declare_bool("racing_simulator.visualize_abscissa"),
          declare_bool("racing_simulator.visualize_vehicle"),
          declare_string("racing_simulator.race_track_file_path"),
          casadi::DM(declare_vec("racing_simulator.x0"))
        }
  );
}
}  // namespace racing_simulator
}  // namespace simulation
}  // namespace lmpc
