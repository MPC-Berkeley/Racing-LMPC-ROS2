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

#include "racing_mpc/racing_mpc.hpp"
#include "racing_mpc/ros_param_loader.hpp"

TEST(RacingMPCTest, RacingMPCTest) {
  rclcpp::init(0, nullptr);
  const auto share_dir = ament_index_cpp::get_package_share_directory("racing_mpc");
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "--params-file", share_dir + "/param/sample_mpc.param.yaml"});
  auto test_node = rclcpp::Node("test_racing_mpc_node", options);

  auto config = lmpc::mpc::racing_mpc::load_parameters(&test_node);
  auto model = lmpc::mpc::racing_mpc::RacingMPC(config);

  rclcpp::shutdown();
  SUCCEED();
}
