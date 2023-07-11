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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "base_vehicle_model/base_vehicle_model.hpp"
#include "base_vehicle_model/ros_param_loader.hpp"

TEST(BaseVehicleModelTest, BaseVehicleModelTest) {
  rclcpp::init(0, nullptr);
  const auto share_dir = ament_index_cpp::get_package_share_directory("base_vehicle_model");
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "--params-file", share_dir + "/param/sample_vehicle.param.yaml"});
  auto test_node = rclcpp::Node("test_base_vehicle_model_node", options);

  auto config = lmpc::vehicle_model::base_vehicle_model::load_parameters(&test_node);
  auto model = lmpc::vehicle_model::base_vehicle_model::BaseVehicleModel(config);

  rclcpp::shutdown();
  SUCCEED();
}
