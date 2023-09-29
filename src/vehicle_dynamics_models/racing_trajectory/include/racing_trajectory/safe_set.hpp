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

#ifndef RACING_TRAJECTORY__SAFE_SET_HPP_
#define RACING_TRAJECTORY__SAFE_SET_HPP_

#include <memory>
#include <vector>
#include <string>
#include <shared_mutex>

#include <boost/circular_buffer.hpp>
#include <casadi/casadi.hpp>

#include "racing_trajectory/trajectory_kd_tree.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
struct SSQuery
{
  casadi::DM x;    // state query
  casadi::DM dist_max;    // maximum distance to the safe set
  casadi_int max_num_total;    // maximum number of points to return
  casadi_int max_num_per_lap;    // maximum number of points to return per lap
};

struct SSResult
{
  casadi::DM x;    // query results
  casadi::DM u;
  casadi::DM J;    // costs at the query results
};

struct SSTrajectoryData
{
  casadi::DM x;
  casadi::DM u;
  casadi::DM k;
  casadi::DM dt;

  casadi::DM x_repeat;
  casadi::DM J;
};

struct RegQuery
{
  typedef std::vector<std::vector<casadi_int>> Indices;
  casadi::DM x;  // state query
  casadi::DM A;  // from nominal model
  casadi::DM B;  // from nominal model
  casadi::DM C;  // from nominal model
  casadi::Function f;  // nominal model
  double dist_max;  // maximum distance to the safe set
  casadi_int max_num_total;  // maximum number of points to return
  casadi_int max_num_per_lap;  // maximum number of points to return per lap
  Indices reg_in_state_idxs;  // indices of the state variables to be used for regression
  Indices reg_in_control_idxs;  // indices of the control variables to be used for regression
  Indices reg_out_state_idxs;  // indices of the output variables of the regression
};

struct RegResult
{
  casadi::DM A;  // after error dynamics regression
  casadi::DM B;  // after error dynamics regression
  casadi::DM C;  // after error dynamics regression
  casadi::DM xs;  // query results
  casadi::DM us;  // control results
  casadi::DM xip1s;  // next state results
  casadi::DM dists;  // distances to the query results
  casadi::DM ks;  // curvature results
  casadi::DM dts;  // time step results
};

class SSTrajectory
{
public:
  typedef std::shared_ptr<SSTrajectory> SharedPtr;
  typedef std::unique_ptr<SSTrajectory> UniquePtr;

  explicit SSTrajectory(const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t, const double & total_length);

  SSResult query(const SSQuery & query) const;
  std::vector<RegResult> query(const RegQuery & query) const;

private:
  SSTrajectoryData lap_;
  lmpc::vehicle_model::racing_trajectory::TrajectoryKDTree tree_;

  SSTrajectoryData process_lap_data(const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t, const double & total_length) const;
};

class SafeSetManager
{
public:
  typedef std::shared_ptr<SafeSetManager> SharedPtr;
  typedef std::unique_ptr<SafeSetManager> UniquePtr;

  explicit SafeSetManager(const size_t & max_lap_stored);

  void add_lap(const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t, const double & total_length);
  SSResult query(const SSQuery & query);
  RegResult query(const RegQuery & query);

private:
  boost::circular_buffer<SSTrajectory::UniquePtr> laps_;
  std::shared_mutex mutex_;
};

class SafeSetRecorder
{
public:
  typedef std::shared_ptr<SafeSetRecorder> SharedPtr;
  typedef std::unique_ptr<SafeSetRecorder> UniquePtr;

  explicit SafeSetRecorder(
    SafeSetManager & manager,
    const bool & to_file,
    const std::string & file_prefix);
  void step(
    const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t,
    const double & total_length);

  void load(const std::vector<std::string> & from_files, const double & total_length);

private:
  SafeSetManager & manager_;
  casadi::DM last_x_;
  casadi::DM last_u_;
  casadi::DM last_t_;
  casadi::DM last_k_;
  bool last_x_valid_;
  bool initialized_;
  bool to_file_;
  std::string file_prefix_;
  size_t lap_count_;
};

}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // RACING_TRAJECTORY__SAFE_SET_HPP_
