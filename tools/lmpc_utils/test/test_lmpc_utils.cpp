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

#include "lmpc_utils/lookup.hpp"
#include "lmpc_utils/ros_param_helper.hpp"

class DummyTestNode : public rclcpp::Node
{
public:
  explicit DummyTestNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("test_lmpc_utils_node", options) {}
};

TEST(LmpcUtilsTest, RosParamHelperTest) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-p", "test_double:=1.0", "-p", "test_int:=1", "-p",
      "test_vec:=[3.14,2.718]"});
  auto test_node = DummyTestNode(options);

  EXPECT_DOUBLE_EQ(lmpc::utils::declare_parameter<double>(&test_node, "test_double"), 1.0);
  EXPECT_EQ(lmpc::utils::declare_parameter<int64_t>(&test_node, "test_int"), 1);
  EXPECT_NO_THROW(lmpc::utils::declare_parameter<std::vector<double>>(&test_node, "test_vec"));

  EXPECT_THROW(
    lmpc::utils::declare_parameter<double>(
      &test_node,
      "test_double"),
    rclcpp::exceptions::ParameterAlreadyDeclaredException);
  EXPECT_ANY_THROW(
    lmpc::utils::declare_parameter<double>(&test_node, "test_doesnt_exist"));
  EXPECT_ANY_THROW(
    lmpc::utils::declare_parameter<double>(&test_node, "test_int"));
  EXPECT_ANY_THROW(
    lmpc::utils::declare_parameter<double>(&test_node, "test_name_invalid!@#"));

  rclcpp::shutdown();
  SUCCEED();
}
