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

#include <math.h>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "base_vehicle_model/ros_param_loader.hpp"
#include "double_track_planar_model/ros_param_loader.hpp"
#include "racing_mpc/racing_mpc.hpp"
#include "racing_mpc/ros_param_loader.hpp"

using lmpc::mpc::racing_mpc::RacingMPC;
using lmpc::vehicle_model::double_track_planar_model::DoubleTrackPlanarModel;
using lmpc::vehicle_model::double_track_planar_model::XIndex;
using lmpc::vehicle_model::double_track_planar_model::UIndex;

const auto share_dir = ament_index_cpp::get_package_share_directory("racing_mpc");

RacingMPC::SharedPtr get_mpc()
{
  rclcpp::init(0, nullptr);
  const auto base_share_dir = ament_index_cpp::get_package_share_directory("base_vehicle_model");
  const auto model_share_dir = ament_index_cpp::get_package_share_directory(
    "double_track_planar_model");
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "--params-file", base_share_dir + "/param/sample_vehicle_2.param.yaml",
    "--params-file", model_share_dir + "/param/sample_vehicle_2.param.yaml",
    "--params-file", share_dir + "/param/sample_mpc_2.param.yaml"
  });
  auto test_node = rclcpp::Node("test_racing_mpc_node", options);

  auto base_config = lmpc::vehicle_model::base_vehicle_model::load_parameters(&test_node);
  auto model_config = lmpc::vehicle_model::double_track_planar_model::load_parameters(&test_node);
  auto model = std::make_shared<DoubleTrackPlanarModel>(base_config, model_config);

  auto config = lmpc::mpc::racing_mpc::load_parameters(&test_node);
  auto mpc = std::make_shared<RacingMPC>(config, model);

  rclcpp::shutdown();
  return mpc;
}

TEST(RacingMPCTest, RacingMPCSolveTest) {
  using casadi::DM;
  using casadi::Slice;
  auto mpc = get_mpc();
  const auto N = static_cast<casadi_int>(mpc->get_config().N);
  const auto test_data =
    DM::from_file(share_dir + "/test_data/mgkt_turn_4.txt", "txt")(Slice(0, N), Slice()).T();
  const auto bound_left = test_data(Slice(9, 11), Slice());
  const auto bound_right = test_data(Slice(11, 13), Slice());

  const auto X_optm_ref =
    DM::from_file(share_dir + "/test_data/x_optm.txt", "txt")(Slice(0, N), Slice()).T();
  const auto U_optm_ref =
    DM::from_file(share_dir + "/test_data/u_optm.txt", "txt")(Slice(0, N - 1), Slice()).T();
  const auto T_optm_ref =
    DM::from_file(share_dir + "/test_data/t_optm.txt", "txt")(Slice(0, N - 1), Slice()).T();
  auto sol_in = casadi::DMDict{
    {"X_optm_ref", X_optm_ref},
    {"U_optm_ref", U_optm_ref(Slice(0, 3), Slice())},
    {"T_optm_ref", T_optm_ref},
    {"X_ref", X_optm_ref},
    {"U_ref", U_optm_ref(Slice(0, 3), Slice())},
    {"T_ref", T_optm_ref},
    {"bound_left", bound_left},
    {"bound_right", bound_right}
  };
  auto x_ic = DM(sol_in["X_ref"](Slice(), 0));
  x_ic(0) += 0.3;
  x_ic(1) += 0.3;
  x_ic(5) *= 1.1;
  sol_in["x_ic"] = x_ic;
  sol_in["x_g"] = sol_in["X_ref"](Slice(), -1);
  sol_in["bound_left"] = bound_left;
  sol_in["bound_right"] = bound_right;
  sol_in["Gamma_y_optm_ref"] = U_optm_ref(3, Slice());

  const auto & X_ref = sol_in["X_ref"];
  const auto & V = X_ref(XIndex::V, Slice());
  for (int i = 0; i < V.size2(); i++) {
    ASSERT_TRUE(static_cast<double>(V(i)) != 0.0 && !isnan(static_cast<double>(V(i))));
  }
  auto sol_out = casadi::DMDict{};
  for (int i = 0; i < 10; i++) {
    const auto start = std::chrono::high_resolution_clock::now();
    mpc->solve(sol_in, sol_out);
    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "MPC Execution Time: " << duration.count() << "ms" << std::endl;
    sol_in["X_optm_ref"] = sol_out["X_optm"];
    sol_in["U_optm_ref"] = sol_out["U_optm"];
    sol_in["T_optm_ref"] = sol_out["T_optm"];
    sol_in["Gamma_y_optm_ref"] = sol_out["Gamma_y_optm"];
  }
  sol_out["X_optm"].T().to_file("X_optm.txt", "txt");
  sol_out["U_optm"].T().to_file("U_optm.txt", "txt");
  sol_out["T_optm"].T().to_file("T_optm.txt", "txt");
  sol_out["Gamma_y_optm"].T().to_file("Gamma_y_optm.txt", "txt");
  SUCCEED();
}
