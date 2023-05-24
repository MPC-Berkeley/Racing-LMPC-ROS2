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

#include "double_track_planar_model/ros_param_loader.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace double_track_planar_model
{
DoubleTrackPlanarModelConfig::SharedPtr load_parameters(rclcpp::Node * node)
{
  auto declare_double = [&](const char * name) {
      return lmpc::utils::declare_parameter<double>(node, name);
    };

  return std::make_shared<DoubleTrackPlanarModelConfig>(
    DoubleTrackPlanarModelConfig{
          declare_double("double_track_planar.fd_max"),
          declare_double("double_track_planar.fb_max"),
          declare_double("double_track_planar.td"),
          declare_double("double_track_planar.tb"),
          declare_double("double_track_planar.v_max"),
          declare_double("double_track_planar.p_max"),
          declare_double("double_track_planar.kroll_f"),
          declare_double("double_track_planar.mu")
        }
  );
}
}  // namespace double_track_planar_model
}  // namespace vehicle_model
}  // namespace lmpc
