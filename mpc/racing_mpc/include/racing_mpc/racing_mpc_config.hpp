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

#include <casadi/casadi.hpp>

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
struct RacingMPCConfig
{
  typedef std::shared_ptr<RacingMPCConfig> SharedPtr;

  // optimizer settings
  double max_wall_time;  // max solving time (s)
  double tol;  // convergence tolarance
  double constr_viol_tol;  // constraint violation tolerance

  // constraint settings
  size_t N;  // steps
  double margin;  // safety margin to obstacle (m)
  double average_track_width;  // averange track width for scaling the variables

  // MPC settings
  casadi::DM Q;  // state cost-to-go
  casadi::DM R;  // control cost-to-go
  casadi::DM Qf;  // final state cost
  casadi::DM x_max;  // primal upper bound
  casadi::DM x_min;  // primal lower bound
  casadi::DM u_max;  // primal upper bound
  casadi::DM u_min;  // primal lower bound
};
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_MPC__RACING_MPC_CONFIG_HPP_
