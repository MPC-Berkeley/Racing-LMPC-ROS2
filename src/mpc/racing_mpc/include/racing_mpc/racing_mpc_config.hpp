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

#ifndef RACING_MPC__RACING_MPC_CONFIG_HPP_
#define RACING_MPC__RACING_MPC_CONFIG_HPP_

#include <memory>
#include <vector>
#include <string>

#include <casadi/casadi.hpp>

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
enum RacingMPCStepMode
{
  STEP,
  CONTINUOUS
};

struct RacingMPCConfig
{
  typedef std::shared_ptr<RacingMPCConfig> SharedPtr;

  // optimizer settings
  double max_cpu_time;  // max solving time (s)
  int64_t max_iter;  // max solver iterations
  double tol;  // convergence tolarance

  // constraint settings
  size_t N;  // steps
  double margin;  // safety margin to obstacle (m)
  double average_track_width;  // averange track width for scaling the variables
  bool verbose;  // print debug
  bool jit;  // use jit compilation
  RacingMPCStepMode step_mode = RacingMPCStepMode::STEP;

  // MPC settings
  casadi::DM q_contour;  // contour (lateral error) cost-to-go
  casadi::DM q_heading;  // heading cost-to-go
  casadi::DM q_vel;  // velocity cost-to-go
  casadi::DM q_vy;  // lateral velocity cost-to-go
  casadi::DM q_vyaw;  // yaw rate cost-to-go
  casadi::DM q_boundary;  // boundary slack cost
  casadi::DM R;  // control cost-to-go
  casadi::DM R_d;  // control rate cost-to-go
  casadi::DM x_max;  // primal upper bound
  casadi::DM x_min;  // primal lower bound
  casadi::DM u_max;  // primal upper bound
  casadi::DM u_min;  // primal lower bound
  double max_vel_ref_diff;  // max velocity reference difference

  // LMPC settings
  bool learning;
  casadi::DM convex_hull_slack;
  casadi_int num_ss_pts;
  casadi_int num_ss_pts_per_lap;
  casadi_int max_lap_stored;

  // recording
  bool record;
  std::string path_prefix;

  bool load;
  std::vector<std::string> load_path;
};
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_MPC__RACING_MPC_CONFIG_HPP_
