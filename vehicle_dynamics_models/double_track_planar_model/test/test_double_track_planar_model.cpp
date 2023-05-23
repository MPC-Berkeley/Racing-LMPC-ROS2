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

#include "base_vehicle_model/ros_param_loader.hpp"
#include "double_track_planar_model/ros_param_loader.hpp"
#include "double_track_planar_model/double_track_planar_model.hpp"

TEST(DoubleTrackPlanarModelTest, TestDoubleTrackPlanarModel) {
  rclcpp::init(0, nullptr);
  const auto base_share_dir = ament_index_cpp::get_package_share_directory("base_vehicle_model");
  const auto share_dir = ament_index_cpp::get_package_share_directory("double_track_planar_model");
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "--params-file", base_share_dir + "/param/sample_vehicle.param.yaml",
    "--params-file", share_dir + "/param/sample_vehicle.param.yaml",
  });
  auto test_node = rclcpp::Node("test_double_track_planar_model_node", options);

  auto base_config = lmpc::vehicle_model::base_vehicle_model::load_parameters(&test_node);
  auto config = lmpc::vehicle_model::double_track_planar_model::load_parameters(&test_node);
  auto model = lmpc::vehicle_model::double_track_planar_model::DoubleTrackPlanarModel(
    base_config,
    config);

  rclcpp::shutdown();
  SUCCEED();
}
