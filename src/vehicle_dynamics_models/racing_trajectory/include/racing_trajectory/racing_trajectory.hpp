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

#ifndef RACING_TRAJECTORY__RACING_TRAJECTORY_HPP_
#define RACING_TRAJECTORY__RACING_TRAJECTORY_HPP_

#include <memory>
#include <string>

#include <casadi/casadi.hpp>

#include <racing_trajectory/trajectory_kd_tree.hpp>
#include <lmpc_utils/primitives.hpp>

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
/**
 * @brief Index of the trajectory table columns.
 *
 */
enum TrajectoryIndex : uint8_t
{
  PX = 0,
  PY = 1,
  PZ = 2,
  YAW = 3,
  SPEED = 4,
  CURVATURE = 5,
  DIST_TO_SF_BWD = 6,  // abscissa
  DIST_TO_SF_FWD = 7,
  REGION = 8,
  LEFT_BOUND_X = 9,
  LEFT_BOUND_Y = 10,
  RIGHT_BOUND_X = 11,
  RIGHT_BOUND_Y = 12,
  BANK = 13,
  LON_ACC = 14,
  LAT_ACC = 15,
  TIME = 16
};

/**
 * @brief A class that stores a racing trajectory and provides conversion
 * between global and frenet coordinates.
 * It also provides interpolation of the trajectory and its boundaries.
 *
 */
class RacingTrajectory
{
public:
  typedef std::shared_ptr<RacingTrajectory> SharedPtr;
  typedef std::unique_ptr<RacingTrajectory> UniquePtr;

  explicit RacingTrajectory(const casadi::DM & traj);
  explicit RacingTrajectory(const std::string & file_name);

  /**
   * @brief Convert a frenet coordinate to a global coordinate.
   *
   * @param frenet_pose input frenet pose.
   * @param global_pose output global pose.
   */
  void frenet_to_global(const FrenetPose2D & frenet_pose, Pose2D & global_pose);

  /**
   * @brief Convert a global coordinate to a frenet coordinate.
   *
   * @param global_pose input global pose.
   * @param frenet_pose output frenet pose.
   * @param initialize_with_previous if true, the previous frenet pose supplied here will be used as initial guess.
   */
  void global_to_frenet(
    const Pose2D & global_pose, FrenetPose2D & frenet_pose,
    const bool & initialize_with_previous = false
  );

  /**
   * @brief Exposes the frenet-global conversion function.
   * Could be used for efficient evaluation and possible gradient evaluation.
   * Takes a vector {s, t, xi} as input and returns a vector {x, y, theta}.
   *
   * @return casadi::Function& the frenet-global conversion function.
   */
  casadi::Function & frenet_to_global_function();

  /**
   * @brief Exposes the global-frenet conversion function.
   * Takes a vector {x, y, theta, s0, t0} as input and returns a vector {s, t, xi}.
   * Less useful than the frenet_to_global_function() because it requires the initial guesses s0 and t0.
   * This function is not differentiable since it uses a qp solver.
   *
   * @return casadi::Function&
   */
  casadi::Function & global_to_frenet_function();

  /**
   * @brief Exposes the curvature interpolation function.
   * Takes a abscissa as input and returns the interpolated curvature.
   *
   * @return casadi::Function&
   */
  casadi::Function & curvature_interpolation_function();

  /**
   * @brief Exposes the left boundary interpolation function.
   * Takes a abscissa as input and returns the interpolated left boundary.
   *
   * @return casadi::Function&
   */
  casadi::Function & left_boundary_interpolation_function();

  /**
   * @brief Exposes the right boundary interpolation function.
   * Takes a abscissa as input and returns the interpolated right boundary.
   *
   * @return casadi::Function&
   */
  casadi::Function & right_boundary_interpolation_function();

  /**
   * @brief Exposes the x interpolation function.
   * Takes a abscissa as input and returns the interpolated global x.
   *
   * @return casadi::Function&
   */
  casadi::Function & x_interpolation_function();

  /**
   * @brief Exposes the y interpolation function.
   * Takes a abscissa as input and returns the interpolated global y.
   *
   * @return casadi::Function&
   */
  casadi::Function & y_interpolation_function();

  /**
   * @brief Exposes the yaw interpolation function.
   * Takes a abscissa as input and returns the interpolated yaw.
   *
   * @return casadi::Function&
   */
  casadi::Function & yaw_interpolation_function();

  /**
   * @brief Exposes the velocity interpolation function.
   * Takes a abscissa as input and returns the interpolated velocity.
   *
   * @return casadi::Function&
   */
  casadi::Function & velocity_interpolation_function();

  const double & total_length() const;

protected:
  casadi::DM traj_;  // stores the trajectory table
  casadi::DM abscissa_;  // stores the abscissa copied from the trajectory table
  casadi::Function norm_2_;  // helper function to compute the norm of all waypoints
  casadi::Function yaw_intp_;  // interpolate yaw
  casadi::Function curvature_intp_;  // interpolate curvature
  casadi::Function left_intp_;  // interpolate left boundary
  casadi::Function right_intp_;  // interpolate right boundary
  casadi::Function x_intp_;  // interpolate global x
  casadi::Function y_intp_;  // interpolate global y
  casadi::Function vel_intp_;  // interpolate velocity
  casadi::Function frenet_to_global_;  // frenet to global conversion function
  casadi::Function global_to_frenet_;  // global to frenet conversion function
  casadi::Function global_to_frenet_sol_;  // g_to_f qp solver

  double total_length_;  // total length of the trajectory

  // kd tree for fast nearest neighbor search of global coordinates
  TrajectoryKDTree kd_tree_;
};
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // RACING_TRAJECTORY__RACING_TRAJECTORY_HPP_
