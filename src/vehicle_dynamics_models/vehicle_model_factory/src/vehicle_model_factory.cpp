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

#include "vehicle_model_factory/vehicle_model_factory.hpp"
#include "base_vehicle_model/ros_param_loader.hpp"
#include "kinematic_bicycle_model/kinematic_bicycle_model.hpp"
#include "kinematic_bicycle_model/ros_param_loader.hpp"
#include "single_track_planar_model/single_track_planar_model.hpp"
#include "single_track_planar_model/ros_param_loader.hpp"
#include "double_track_planar_model/double_track_planar_model.hpp"
#include "double_track_planar_model/ros_param_loader.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace vehicle_model_factory
{
base_vehicle_model::BaseVehicleModel::SharedPtr load_vehicle_model(
  const std::string model_name,
  rclcpp::Node * node)
{
  const auto base_config = lmpc::vehicle_model::base_vehicle_model::load_parameters(node);
  if (model_name == "kinematic_bicycle_model") {
    const auto config = lmpc::vehicle_model::kinematic_bicycle_model::load_parameters(node);
    return std::make_shared<kinematic_bicycle_model::KinematicBicycleModel>(base_config, config);
  } else if (model_name == "single_track_planar_model") {
    const auto config = lmpc::vehicle_model::single_track_planar_model::load_parameters(node);
    return std::make_shared<single_track_planar_model::SingleTrackPlanarModel>(base_config, config);
  } else if (model_name == "double_track_planar_model") {
    const auto config = lmpc::vehicle_model::double_track_planar_model::load_parameters(node);
    return std::make_shared<double_track_planar_model::DoubleTrackPlanarModel>(base_config, config);
  } else {
    RCLCPP_FATAL(node->get_logger(), "Vehicle model %s cannot be found.", model_name.c_str());
    return nullptr;
  }
}
}  // namespace vehicle_model_factory
}  // namespace vehicle_model
}  // namespace lmpc
