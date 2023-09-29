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

#include <memory>
#include <vector>
#include <algorithm>
#include <execution>

#include <casadi/casadi.hpp>
#include <Eigen/Dense>

#include "racing_trajectory/safe_set.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
SSTrajectory::SSTrajectory(const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t, const double & total_length)
: lap_(process_lap_data(x, u, k, t, total_length)),
  tree_(lap_.x_repeat(0, casadi::Slice()).get_elements(),
    lap_.x_repeat(1, casadi::Slice()).get_elements())
{
}

SSResult SSTrajectory::query(const SSQuery & query) const
{
  SSResult result;
  std::vector<size_t> indices;
  indices.reserve(query.max_num_per_lap);
  tree_.find_closest_waypoint_indices(
    static_cast<double>(query.x(0)),
    static_cast<double>(query.x(1)),
    query.max_num_per_lap, indices);
  result.x = lap_.x_repeat(casadi::Slice(), indices);
  result.J = lap_.J(casadi::Slice(), indices);
  return result;
}

std::vector<RegResult> SSTrajectory::query(const RegQuery & query) const
{
  std::vector<RegResult> results;

  for (casadi_int i = 0; i < query.reg_out_state_idxs.size(); i++)
  {
    auto & result = results.emplace_back();
    if (query.reg_out_state_idxs[i].size() != 1)
    {
      throw std::invalid_argument("Only one state variable is supported in every regression");
    }

    // slice the x matrix to get the regression input
    // the last x is not considered since no xip1 is available for it
    casadi::DM x_data = lap_.x(query.reg_in_state_idxs[i], casadi::Slice(0, -1));
    casadi::DM u_data = lap_.u(query.reg_in_control_idxs[i], casadi::Slice(0, -1));
    casadi::DM xip1_data = lap_.x(query.reg_in_state_idxs[i], casadi::Slice(1, std::numeric_limits<casadi_int>::max()));
    casadi::DM k_data = lap_.k(casadi::Slice(), casadi::Slice(0, -1));
    casadi::DM dt_data = lap_.dt(casadi::Slice(), casadi::Slice(0, -1));

    // stack x and u
    casadi::DM z_data = casadi::DM::vertcat({x_data, u_data});
    // compute distance to the query point. sort the points by distance.
    result.dists = casadi::DM::sqrt(
      casadi::DM::sum1(
        casadi::DM::pow(
          z_data - casadi::DM::repmat(query.x, 1, z_data.size2()), 2)));
    // create a mask to filter out the points that are too far away
    const auto mask = result.dists < query.dist_max;
    x_data = x_data(casadi::Slice(), mask);
    u_data = u_data(casadi::Slice(), mask);
    xip1_data = xip1_data(casadi::Slice(), mask);
    k_data = k_data(casadi::Slice(), mask);
    dt_data = dt_data(casadi::Slice(), mask);
    result.dists = result.dists(mask);
    // if there are no points left, skip the sort
    if (x_data.size2() == 0) {
      continue;
    }

    // convert to eigen and compute the sorted indices
    auto dist_vec = result.dists.get_elements();
    Eigen::Map<Eigen::VectorXd> dist_eigen(dist_vec.data(), dist_vec.size());
    Eigen::VectorXi idx = Eigen::VectorXi::LinSpaced(dist_vec.size(), 0, dist_vec.size() - 1);
    std::sort(idx.data(), idx.data() + idx.size(),
      [&dist_eigen](size_t i1, size_t i2) {return dist_eigen(i1) < dist_eigen(i2);});
    // convert idx to std vector
    std::vector<casadi_int> idx_vec(idx.data(), idx.data() + idx.size());
    // slice the data according to the sorted indices
    result.xs = x_data(casadi::Slice(), idx_vec);
    result.us = u_data(casadi::Slice(), idx_vec);
    result.xip1s = xip1_data(casadi::Slice(), idx_vec);
    result.ks = k_data(casadi::Slice(), idx_vec);
    result.dts = dt_data(casadi::Slice(), idx_vec);
  }
  return results;
}

SSTrajectoryData SSTrajectory::process_lap_data(const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t, const double & total_length) const
{
  SSTrajectoryData data;
  const auto J = casadi::DM::linspace(x.size2() - 1, 0, x.size2()).T();
  auto x_offset = casadi::DM::zeros(x.size1(), x.size2());
  x_offset(0, casadi::Slice()) = total_length;
  data.x_repeat = casadi::DM::horzcat({x - x_offset, x, x + x_offset});
  data.u = u;
  data.k = k;
  data.J = casadi::DM::horzcat({J + x.size2() - 1, J, J - x.size2() + 1});
  data.x = x;
  data.dt = t(casadi::Slice(), casadi::Slice(0, -1)) - t(casadi::Slice(), casadi::Slice(1, std::numeric_limits<casadi_int>::max()));
  data.dt = casadi::DM::horzcat({data.dt, data.dt(casadi::Slice(), -1)});
  return data;
}

SafeSetManager::SafeSetManager(const size_t & max_lap_stored)
: laps_(max_lap_stored)
{
}

void SafeSetManager::add_lap(const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t, const double & total_length)
{
  auto traj = std::make_unique<SSTrajectory>(x, u, k, t, total_length);
  std::unique_lock<std::shared_mutex> lock(mutex_);
  laps_.push_back(std::move(traj));
}

SSResult SafeSetManager::query(const SSQuery & query)
{
  SSResult result;
  casadi::DMVector x;
  casadi::DMVector J;
  x.reserve(laps_.size());
  J.reserve(laps_.size());

  casadi_int num_total = 0;
  std::shared_lock<std::shared_mutex> lock(mutex_);
  // iterate from the last lap to the first lap
  for (auto it = laps_.rbegin(); num_total < query.max_num_total && it != laps_.rend(); ++it) {
    const auto result_lap = (*it)->query(query);
    x.push_back(result_lap.x);
    J.push_back(result_lap.J);
    num_total += result_lap.x.size2();
  }
  lock.unlock();

  result.x = casadi::DM::horzcat(x);
  result.J = casadi::DM::horzcat(J);

  if (result.x.size2() > query.max_num_total) {
    result.x = result.x(casadi::Slice(), casadi::Slice(0, query.max_num_total));
    result.J = result.J(casadi::Slice(), casadi::Slice(0, query.max_num_total));
  }
  return result;
}

RegResult SafeSetManager::query(const RegQuery & query)
{
  // parallelly query all the laps
  std::vector<std::vector<RegResult>> results(laps_.size());
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::transform(
    std::execution::par_unseq,
    laps_.begin(), laps_.end(), results.begin(),
    [&query](const auto & lap) {return lap->query(query);});
  lock.unlock();

  // aggregate the results and perform regression
  RegResult result;
  result.A = query.A;
  result.B = query.B;
  result.C = query.C;
  for (casadi_int i = 0; i < query.reg_out_state_idxs.size(); i++)
  {
    // concatenate the results from all the laps
    casadi::DM xs;
    casadi::DM us;
    casadi::DM xip1s;
    casadi::DM dists;
    casadi::DM ks;
    casadi::DM dts;
    for (const auto & lap_result : results) {
      xs = casadi::DM::horzcat({xs, lap_result[i].xs});
      us = casadi::DM::horzcat({us, lap_result[i].us});
      xip1s = casadi::DM::horzcat({xip1s, lap_result[i].xip1s});
      dists = casadi::DM::vertcat({dists, lap_result[i].dists});
      ks = casadi::DM::horzcat({ks, lap_result[i].ks});
      dts = casadi::DM::horzcat({dts, lap_result[i].dts});
    }
    // if there are no points left, skip the regression
    if (xs.size2() == 0) {
      continue;
    }
    // predict the next state
    const auto xip1s_pred = query.f.map(xs.size2())(casadi::DMDict{{"x", xs}, {"u", us}, {"k", ks}, {"dt", dts}}).at("xip1");
    // compute Epanechnikov kernel weights
    const auto K = 0.75 / query.dist_max * casadi::DM::pow(1 - casadi::DM::pow(dists / query.dist_max, 2), 2);
    // compute the regression matrices
    const auto reg_x_data = casadi::DM::horzcat({xs.T(), us.T()});
    const auto reg_y_data = xip1s.T() - xip1s_pred(query.reg_in_state_idxs[i], casadi::Slice()).T();
    const auto M = casadi::DM::horzcat({reg_x_data, casadi::DM::ones(reg_x_data.size1(), 1)});
    const auto Q = M.T() * casadi::DM::diag(K) * M + 1e-3 * casadi::DM::eye(M.size2());
    const auto b = -M.T() * casadi::DM::diag(K) * reg_y_data;
    // solve the regression
    const auto R = casadi::DM::solve(Q, b);
    // extract the regression results
    const auto dA = R(casadi::Slice(0, static_cast<casadi_int>(query.reg_in_state_idxs[i].size())));
    const auto dB = R(casadi::Slice(static_cast<casadi_int>(query.reg_in_state_idxs[i].size()), -1));
    const auto dC = R(-1);
    // update the regression results
    result.A(query.reg_out_state_idxs[i], query.reg_in_state_idxs[i]) += dA;
    result.B(query.reg_out_state_idxs[i], query.reg_in_control_idxs[i]) += dB;
    result.C(query.reg_out_state_idxs[i]) += dC;
  }
  return result;
}

SafeSetRecorder::SafeSetRecorder(
  SafeSetManager & manager,
  const bool & to_file,
  const std::string & file_prefix)
: manager_(manager),
  last_x_valid_(false),
  initialized_(false),
  to_file_(to_file),
  file_prefix_(file_prefix),
  lap_count_(0)
{
}

void SafeSetRecorder::load(const std::vector<std::string> & from_files, const double & total_length)
{
  for (const auto & filename : from_files) {
    try {
      std::cout << "Loading lap from " << filename << std::endl;
      const auto x = casadi::DM::from_file(filename + "_x.txt", "txt").T();
      const auto u = casadi::DM::from_file(filename + "_u.txt", "txt").T();
      const auto k = casadi::DM::from_file(filename + "_k.txt", "txt").T();
      const auto t = casadi::DM::from_file(filename + "_t.txt", "txt").T();
      manager_.add_lap(x, u, k, t, total_length);
      lap_count_++;
    } catch (const std::exception & e) {
      std::cout << "Failed to load lap from " << filename << std::endl;
      std::cout << e.what() << std::endl;
    }
  }
}

void SafeSetRecorder::step(
  const casadi::DM & x, const casadi::DM & u, const casadi::DM & k, const casadi::DM & t,
  const double & total_length)
{
  if (!last_x_valid_) {
    last_x_ = x;
    last_x_valid_ = true;
    return;
  }

  const auto px = static_cast<double>(x(0));
  const auto px_last = static_cast<double>(last_x_(0, -1));
  if (px_last - px > 0.5 * total_length) {
    // new lap
    if (initialized_) {
      std::cout << "Lap " << lap_count_ << " completed. Adding to safe set." << std::endl;
      std::cout << "Lap " << lap_count_ << " ave speed: " <<
        static_cast<double>(total_length / (t - last_t_(0))) << " m/s, time: " <<
        (t - last_t_(0)) << " s." << std::endl;
      manager_.add_lap(last_x_, last_u_, last_k_, last_t_, total_length);
      if (to_file_) {
        const auto filename = file_prefix_ + "lap_" + std::to_string(lap_count_);
        std::cout << "Saving lap to " << filename << std::endl;
        last_x_.T().to_file(filename + "_x.txt", "txt");
        last_u_.T().to_file(filename + "_u.txt", "txt");
        last_t_.T().to_file(filename + "_t.txt", "txt");
        last_k_.T().to_file(filename + "_k.txt", "txt");
      }
      std::cout << "------------------------------------------------------------" << std::endl;
    } else {
      initialized_ = true;
    }
    lap_count_++;
    std::cout << "Beginning recording lap " << lap_count_ << std::endl;
    last_x_ = x;
    last_u_ = u;
    last_t_ = t;
    last_k_ = k;
  } else {
    last_x_ = casadi::DM::horzcat({last_x_, x});
    last_u_ = casadi::DM::horzcat({last_u_, u});
    last_t_ = casadi::DM::horzcat({last_t_, t});
    last_k_ = casadi::DM::horzcat({last_k_, k});
  }
}

}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
