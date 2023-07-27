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

#include <base_vehicle_model/ros_param_loader.hpp>
#include <single_track_planar_model/ros_param_loader.hpp>
#include <lmpc_utils/primitives.hpp>
#include <racing_trajectory/racing_trajectory.hpp>
#include "racing_lmpc/racing_lmpc.hpp"
#include "racing_lmpc/ros_param_loader.hpp"

using lmpc::mpc::racing_lmpc::RacingLMPC;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;
using lmpc::vehicle_model::single_track_planar_model::XIndex;
using lmpc::vehicle_model::single_track_planar_model::UIndex;

const auto share_dir = ament_index_cpp::get_package_share_directory("racing_lmpc");
const auto base_share_dir = ament_index_cpp::get_package_share_directory("base_vehicle_model");
const auto model_share_dir = ament_index_cpp::get_package_share_directory(
  "single_track_planar_model");
const auto trajectory_share_dir = ament_index_cpp::get_package_share_directory(
  "racing_trajectory");
RacingLMPC::SharedPtr get_mpc()
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "--params-file", base_share_dir + "/param/sample_vehicle_2.param.yaml",
    "--params-file", model_share_dir + "/param/sample_vehicle_2.param.yaml",
    "--params-file", share_dir + "/param/sample_mpc_2.param.yaml"
  });
  auto test_node = rclcpp::Node("test_racing_lmpc_node", options);

  auto base_config = lmpc::vehicle_model::base_vehicle_model::load_parameters(&test_node);
  auto model_config = lmpc::vehicle_model::single_track_planar_model::load_parameters(&test_node);
  auto model = std::make_shared<SingleTrackPlanarModel>(base_config, model_config);

  auto config = lmpc::mpc::racing_lmpc::load_parameters(&test_node);
  auto mpc = std::make_shared<RacingLMPC>(config, model);

  rclcpp::shutdown();
  return mpc;
}

void test_mpc(const lmpc::Pose2D & p0, const double & v0, const casadi_int & num_step)
{
  using casadi::DM;
  using casadi::Slice;
  auto mpc = get_mpc();
  const auto N = static_cast<casadi_int>(mpc->get_config().N);

  // load the test trajectory
  const auto test_traj_file = trajectory_share_dir + "/test_data/mgkt_optm.txt";
  auto traj = lmpc::vehicle_model::racing_trajectory::RacingTrajectory(test_traj_file);

  // loop
  // create the initial reference
  auto X_optm_ref = DM::zeros(mpc->get_model().nx(), N);
  const auto U_optm_ref = DM::zeros(mpc->get_model().nu(), N - 1);
  const auto T_optm_ref = DM::zeros(1, N - 1) + 0.1;

  lmpc::FrenetPose2D x0_frenet;
  traj.global_to_frenet(p0, x0_frenet);

  const auto x_ic = DM{
    x0_frenet.position.s, x0_frenet.position.t, x0_frenet.yaw,
    v0, 0.0, 0.0
  };

  X_optm_ref(XIndex::PX, 0) = x0_frenet.position.s;
  X_optm_ref(XIndex::VX, 0) = v0;

  for (int i = 1; i < N; i++) {
    X_optm_ref(XIndex::PX, i) = X_optm_ref(XIndex::PX, i - 1) + 0.1 * v0;
    X_optm_ref(XIndex::VX, i) = v0;
  }

  const auto total_length = traj.total_length();

  auto sol_in = casadi::DMDict{
    {"X_optm_ref", X_optm_ref},
    {"U_optm_ref", U_optm_ref(Slice(0, 3), Slice())},
    {"T_optm_ref", T_optm_ref},
    {"X_ref", X_optm_ref},
    {"U_ref", U_optm_ref(Slice(0, 3), Slice())},
    {"T_ref", T_optm_ref},
    {"total_length", total_length},
    {"x_ic", x_ic}
  };

  for (int i = 0; i < num_step; i++) {
    const auto left_ref = traj.left_boundary_interpolation_function()(
      X_optm_ref(
        XIndex::PX,
        Slice()))[0];
    const auto right_ref =
      traj.right_boundary_interpolation_function()(X_optm_ref(XIndex::PX, Slice()))[0];
    const auto curvature_ref =
      traj.curvature_interpolation_function()(X_optm_ref(XIndex::PX, Slice()))[0];
    const auto vel_ref = traj.velocity_interpolation_function()(X_optm_ref(XIndex::PX, Slice()))[0];
    sol_in["bound_left"] = left_ref;
    sol_in["bound_right"] = right_ref;
    sol_in["curvatures"] = curvature_ref;
    sol_in["vel_ref"] = vel_ref;


    auto sol_out = casadi::DMDict{};
    auto stats = casadi::Dict{};
    const auto start = std::chrono::high_resolution_clock::now();
    mpc->solve(sol_in, sol_out, stats);
    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "MPC Execution Time: " << duration.count() << "ms" << std::endl;
    if (mpc->solved()) {
      sol_in.erase("X_optm_ref");
      sol_in.erase("U_optm_ref");
      sol_in.erase("T_optm_ref");
    }

    auto X_optm_out = sol_out["X_optm"];
    auto X_optm_out_global = X_optm_out;
    auto f2g = traj.frenet_to_global_function().map(mpc->get_config().N);
    X_optm_out_global(
      Slice(XIndex::PX, XIndex::YAW + 1),
      Slice()) = f2g(X_optm_out(Slice(XIndex::PX, XIndex::YAW + 1), Slice()))[0];

    auto U_optm_out = sol_out["U_optm"];
    X_optm_out_global.T().to_file("test_X_optm.txt", "txt");
    U_optm_out.T().to_file("test_U_optm.txt", "txt");
    T_optm_ref.T().to_file("test_T_optm.txt", "txt");

    X_optm_ref = X_optm_out;
    sol_in["X_ref"] = X_optm_out;
    sol_in["U_ref"] = U_optm_out;

    // teleport the vehicle to the next position
    sol_in["x_ic"] = X_optm_out(Slice(), 1);
  }
  std::cout << sol_in["X_ref"](Slice(), -1) << std::endl;
}

TEST(RacingLMPCTest, MPCSolveTest)
{
  const lmpc::Pose2D x0_pose2d{
    84.83, -112.67, -2.3532
  };
  const double v0 = 10.0;
  const casadi_int num_step = 10;
  test_mpc(x0_pose2d, v0, num_step);
  SUCCEED();
}

TEST(RacingLMPCTest, MPCSolveTestStartDeviated)
{
  const lmpc::Pose2D x0_pose2d{
    85.4, -113.3, -2.3532
  };
  const double v0 = 10.0;
  const casadi_int num_step = 10;
  test_mpc(x0_pose2d, v0, num_step);
  SUCCEED();
}

// TEST(RacingLMPCTest, MPCSolveTestAtJoint)
// {
//   const lmpc::Pose2D x0_pose2d{
//     -6.04, 5.76, -1.11
//   };
//   const double v0 = 20.0;
//   const casadi_int num_step = 10;
//   test_mpc(x0_pose2d, v0, num_step);
//   SUCCEED();
// }
