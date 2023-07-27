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

#include "vanilla_controller/ros_param_loader.hpp"

namespace lmpc
{
namespace mpc
{
namespace vanilla_controller
{
VanillaControllerConfig::SharedPtr load_parameters(rclcpp::Node * node)
{
  auto declare_double = [&](const char * name) {
      return lmpc::utils::declare_parameter<double>(node, name);
    };
  auto declare_string = [&](const char * name) {
      return lmpc::utils::declare_parameter<std::string>(node, name);
    };
  // auto declare_vec = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<std::vector<double>>(node, name);
  //   };
  // auto declare_int = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<int64_t>(node, name);
  //   };
  // auto declare_bool = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<bool>(node, name);
  //   };

  const auto step_mode_str = declare_string("vanilla_controller.step_mode");
  VanillaControllerStepMode step_mode;
  if (step_mode_str == "step") {
    step_mode = VanillaControllerStepMode::STEP;
  } else if (step_mode_str == "continuous") {
    step_mode = VanillaControllerStepMode::CONTINUOUS;
  } else {
    throw std::invalid_argument("Invalid step mode: " + step_mode_str);
  }

  return std::make_shared<VanillaControllerConfig>(
    VanillaControllerConfig{
          declare_double("vanilla_controller.lookahead_speed_ratio"),
          declare_double("vanilla_controller.min_lookahead_distance"),
          declare_double("vanilla_controller.max_lookahead_distance"),
          utils::PidCoefficients{
            declare_double("vanilla_controller.lon_kp"),
            declare_double("vanilla_controller.lon_ki"),
            declare_double("vanilla_controller.lon_kd"),
            declare_double("vanilla_controller.lon_min_acc"),
            declare_double("vanilla_controller.lon_max_acc"),
            declare_double("vanilla_controller.lon_ki_min"),
            declare_double("vanilla_controller.lon_ki_max"),
          },
          declare_double("vanilla_controller.dt"),
          step_mode,
        }
  );
}
}  // namespace vanilla_controller
}  // namespace mpc
}  // namespace lmpc
