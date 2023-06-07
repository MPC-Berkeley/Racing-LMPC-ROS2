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
#include "ekf_state_estimator/ekf_state_estimator.hpp"
#include "ekf_state_estimator/ros_param_loader.hpp"

using lmpc::state_estimator::ekf_state_estimator::EKFStateEstimator;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;
using lmpc::vehicle_model::single_track_planar_model::XIndex;
using lmpc::vehicle_model::single_track_planar_model::UIndex;

const auto share_dir = ament_index_cpp::get_package_share_directory("ekf_state_estimator");

EKFStateEstimator::SharedPtr get_ekf()
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
    "--params-file", share_dir + "/param/sample_ekf.param.yaml"
  });
  auto test_node = rclcpp::Node("test_ekf_state_estimator_node", options);

  auto base_config = lmpc::vehicle_model::base_vehicle_model::load_parameters(&test_node);
  auto model_config = lmpc::vehicle_model::single_track_planar_model::load_parameters(&test_node);
  auto model = std::make_shared<SingleTrackPlanarModel>(base_config, model_config);

  auto config = lmpc::state_estimator::ekf_state_estimator::load_parameters(&test_node);
  auto ekf = std::make_shared<EKFStateEstimator>(config, model);

  rclcpp::shutdown();
  return ekf;
}

TEST(EKFStateEstimatorTest, EKFStateEstimatorSolveTest) {
  using casadi::DM;
  using casadi::Slice;
  auto ekf = get_ekf();
  const auto N = 21;
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
  const auto t_intp = DM::linspace(0.0, 1.0, N);
  const auto bound_left_intp = DM::interp1d(
    t_vec, bound_left.T(),
    t_intp.get_elements(), "", false).T();
  const auto bound_right_intp = DM::interp1d(
    t_vec, bound_right.T(),
    t_intp.get_elements(), "", false).T();
  const auto X_optm_ref_intp =
    DM::interp1d(t_vec, X_optm_ref, t_intp.get_elements(), "", false).T();
  const auto U_optm_ref_intp =
    DM::interp1d(
    T_accum(Slice(0, -1)).get_elements(), U_optm_ref, t_intp(
      Slice(
        0, -1)).get_elements(), "", false).T();
  const auto T_optm_ref_intp = DM::zeros(N - 1) + t_intp(1) - t_intp(0);

  auto sol_in = casadi::DMDict{
    {"X_ref", X_optm_ref_intp},
    {"U_ref", U_optm_ref_intp(Slice(0, 3), Slice())},
    {"T_ref", T_optm_ref_intp}
  };
  auto x_ic = DM(sol_in["X_ref"](Slice(), 0));
  x_ic(XIndex::PX) += 1.0;
  x_ic(XIndex::PY) += 0.3;
  x_ic(XIndex::YAW) += 0.01;
  // x_ic(XIndex::V) *= 1.1;
  sol_in["x_ic"] = x_ic;

  SUCCEED();
}
