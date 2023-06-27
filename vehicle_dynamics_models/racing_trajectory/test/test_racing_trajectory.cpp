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

#include <chrono>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "racing_trajectory/racing_trajectory.hpp"

TEST(RacingTrajectoryTest, TestGlobalToFrenetUninitialized) {
  const auto share_dir = ament_index_cpp::get_package_share_directory("racing_trajectory");
  const auto test_file_dir = share_dir + "/test_data/mgkt_optm.txt";
  auto traj = lmpc::vehicle_model::racing_trajectory::RacingTrajectory(test_file_dir);

  const auto test_global_pose = lmpc::Pose2D{{0.0, 0.0}, -M_PI_2};
  auto test_frenet_pose = lmpc::FrenetPose2D();

  const auto start_time = std::chrono::high_resolution_clock::now();
  traj.global_to_frenet(test_global_pose, test_frenet_pose);
  const auto end_time = std::chrono::high_resolution_clock::now();
  std::cout << "[Test Global to Frenet (Uninitialized)]" << std::endl;
  std::cout << "Global to Frenet took "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count()
            << " microseconds." << std::endl;
  std::cout << "Global pose: " << test_global_pose << std::endl;
  std::cout << "Frenet pose: " << test_frenet_pose << std::endl;
}

TEST(RacingTrajectoryTest, TestGlobalToFrenetInitialized) {
  const auto share_dir = ament_index_cpp::get_package_share_directory("racing_trajectory");
  const auto test_file_dir = share_dir + "/test_data/mgkt_optm.txt";
  auto traj = lmpc::vehicle_model::racing_trajectory::RacingTrajectory(test_file_dir);

  const auto test_global_pose = lmpc::Pose2D{{0.0, 0.0}, -M_PI_2};
  auto test_global_pose_moved = lmpc::Pose2D{{0.5, 0.5}, -M_PI_2};
  auto test_frenet_pose = lmpc::FrenetPose2D();

  traj.global_to_frenet(test_global_pose, test_frenet_pose);

  for (int i = 0; i < 5; i++) {
    const auto start_time = std::chrono::high_resolution_clock::now();
    traj.global_to_frenet(test_global_pose_moved, test_frenet_pose, true);
    const auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "\n[Test Global to Frenet (Initialized)]" << std::endl;
    std::cout << "Global to Frenet took "
              << std::chrono::duration_cast<std::chrono::microseconds>(
      end_time -
      start_time).count()
              << " microseconds." << std::endl;
    std::cout << "Global pose: " << test_global_pose << std::endl;
    std::cout << "Frenet pose: " << test_frenet_pose << std::endl;

    test_global_pose_moved.position.x += 0.1;
    test_global_pose_moved.position.y += 0.5;
  }
}
