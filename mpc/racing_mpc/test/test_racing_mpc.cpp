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
#include "single_track_planar_model/ros_param_loader.hpp"
#include "racing_mpc/racing_mpc.hpp"
#include "racing_mpc/ros_param_loader.hpp"

using lmpc::mpc::racing_mpc::RacingMPC;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;
using lmpc::vehicle_model::single_track_planar_model::XIndex;
using lmpc::vehicle_model::single_track_planar_model::UIndex;

const auto share_dir = ament_index_cpp::get_package_share_directory("racing_mpc");

RacingMPC::SharedPtr get_mpc()
{
  rclcpp::init(0, nullptr);
  const auto base_share_dir = ament_index_cpp::get_package_share_directory("base_vehicle_model");
  const auto model_share_dir = ament_index_cpp::get_package_share_directory(
    "single_track_planar_model");
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
  auto model_config = lmpc::vehicle_model::single_track_planar_model::load_parameters(&test_node);
  auto model = std::make_shared<SingleTrackPlanarModel>(base_config, model_config);

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
  sol_in["x_ic"] = x_ic;
  sol_in["x_g"] = sol_in["X_ref"](Slice(), -1);
  sol_in["bound_left"] = bound_left;
  sol_in["bound_right"] = bound_right;

  const auto & X_ref = sol_in["X_ref"];
  const auto & V = X_ref(XIndex::V, Slice());
  for (int i = 0; i < V.size2(); i++) {
    ASSERT_TRUE(static_cast<double>(V(i)) != 0.0 && !isnan(static_cast<double>(V(i))));
  }
  auto sol_out = casadi::DMDict{};
  for (int i = 0; i < 3; i++) {
    const auto start = std::chrono::high_resolution_clock::now();
    mpc->solve(sol_in, sol_out);
    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "MPC Execution Time: " << duration.count() << "ms" << std::endl;
    sol_in.erase("X_optm_ref");
    sol_in.erase("U_optm_ref");
    sol_in.erase("T_optm_ref");
  }
  sol_out["X_optm"].T().to_file("test_X_optm.txt", "txt");
  sol_out["U_optm"].T().to_file("test_U_optm.txt", "txt");
  sol_out["T_optm"].T().to_file("test_T_optm.txt", "txt");
  SUCCEED();
}

TEST(RacingMPCTest, RacingMPCSolveInterpolatedTest) {
  using casadi::DM;
  using casadi::Slice;
  auto mpc = get_mpc();
  const auto N = static_cast<casadi_int>(mpc->get_config().N);
  const auto test_data =
    DM::from_file(share_dir + "/test_data/mgkt_turn_4.txt", "txt").T();
  const auto bound_left = test_data(Slice(9, 11), Slice());
  const auto bound_right = test_data(Slice(11, 13), Slice());

  const auto X_optm_ref =
    DM::from_file(share_dir + "/test_data/x_optm.txt", "txt");
  const auto U_optm_ref =
    DM::from_file(share_dir + "/test_data/u_optm.txt", "txt");
  const auto T_optm_ref =
    DM::from_file(share_dir + "/test_data/t_optm.txt", "txt");

  auto T_accum = DM::zeros(T_optm_ref.size1() + 1);
  for (int i = 0; i < T_optm_ref.size1(); i++) {
    T_accum(i + 1) = T_accum(i) + T_optm_ref(i);
  }

  const auto t_vec = T_accum.get_elements();
  const auto t_intp = DM::linspace(0.0, 2.0, N);
  const auto bound_left_intp = DM::interp1d(
    t_vec, bound_left.T(),
    t_intp.get_elements(), "floor", false).T();
  const auto bound_right_intp = DM::interp1d(
    t_vec, bound_right.T(),
    t_intp.get_elements(), "floor", false).T();
  const auto X_optm_ref_intp =
    DM::interp1d(t_vec, X_optm_ref, t_intp.get_elements(), "floor", false).T();
  const auto U_optm_ref_intp =
    DM::interp1d(
    T_accum(Slice(0, -1)).get_elements(), U_optm_ref, t_intp(
      Slice(
        0, -1)).get_elements(), "floor", false).T();
  const auto T_optm_ref_intp = DM::zeros(N - 1) + t_intp(1) - t_intp(0);

  auto sol_in = casadi::DMDict{
    {"X_optm_ref", X_optm_ref_intp},
    {"U_optm_ref", U_optm_ref_intp(Slice(0, 3), Slice())},
    {"T_optm_ref", T_optm_ref_intp},
    {"X_ref", X_optm_ref_intp},
    {"U_ref", U_optm_ref_intp(Slice(0, 3), Slice())},
    {"T_ref", T_optm_ref_intp},
    {"bound_left", bound_left_intp},
    {"bound_right", bound_right_intp}
  };
  auto x_ic = DM(sol_in["X_ref"](Slice(), 0));
  x_ic(0) += 0.3;
  x_ic(1) += 0.3;
  sol_in["x_ic"] = x_ic;
  sol_in["x_g"] = sol_in["X_ref"](Slice(), -1);

  auto sol_out = casadi::DMDict{};
  for (int i = 0; i < 10; i++) {
    const auto start = std::chrono::high_resolution_clock::now();
    mpc->solve(sol_in, sol_out);
    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "MPC Execution Time: " << duration.count() << "ms" << std::endl;
    sol_in.erase("X_optm_ref");
    sol_in.erase("U_optm_ref");
    sol_in.erase("T_optm_ref");
  }
  sol_out["X_optm"].T().to_file("test_X_optm.txt", "txt");
  sol_out["U_optm"].T().to_file("test_U_optm.txt", "txt");
  T_optm_ref_intp.T().to_file("test_T_optm.txt", "txt");
  SUCCEED();
}
