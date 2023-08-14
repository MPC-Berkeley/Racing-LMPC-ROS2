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

#include <casadi/casadi.hpp>

#include "racing_trajectory/safe_set.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
SSTrajectory::SSTrajectory(const casadi::DM & x, const double & total_length)
: lap_(process_lap_data(x, total_length)),
  tree_(lap_.x(0, casadi::Slice()).get_elements(),
    lap_.x(1, casadi::Slice()).get_elements())
{
}

SSResult SSTrajectory::query(const SSQuery & query)
{
  SSResult result;
  std::vector<size_t> indices;
  indices.reserve(query.max_num_per_lap);
  tree_.find_closest_waypoint_indices(
    static_cast<double>(query.x(0)),
    static_cast<double>(query.x(1)),
    query.max_num_per_lap, indices);
  result.x = lap_.x(casadi::Slice(), indices);
  result.J = lap_.J(casadi::Slice(), indices);
  return result;
}

SSResult SSTrajectory::process_lap_data(const casadi::DM & x, const double & total_length) const
{
  SSResult result;
  const auto J = casadi::DM::linspace(x.size2() - 1, 0, x.size2()).T();
  auto x_offset = casadi::DM::zeros(x.size1(), x.size2());
  x_offset(0, casadi::Slice()) = total_length;
  result.x = casadi::DM::horzcat({x - x_offset, x, x + x_offset});
  result.J = casadi::DM::horzcat({J + x.size2() - 1, J, J - x.size2() + 1});
  return result;
}

SafeSetManager::SafeSetManager(const size_t & max_lap_stored)
: laps_(max_lap_stored)
{
}

void SafeSetManager::add_lap(const casadi::DM & x, const double & total_length)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  laps_.push_back(std::make_unique<SSTrajectory>(x, total_length));
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
      const auto x = casadi::DM::from_file(filename).T();
      // const auto u = casadi::DM::from_file(filename + "_u.txt", "txt").T();
      // const auto t = casadi::DM::from_file(filename + "_t.txt", "txt").T();
      manager_.add_lap(x, total_length);
      lap_count_++;
    } catch (const std::exception & e) {
      std::cout << "Failed to load lap from " << filename << std::endl;
      std::cout << e.what() << std::endl;
    }
  }
}

void SafeSetRecorder::step(
  const casadi::DM & x, const casadi::DM & u, const casadi::DM & t,
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
        static_cast<double>(total_length / (t - last_t_(0))) << " m/s, time: " << (t - last_t_(0)) << " s." << std::endl;
      manager_.add_lap(last_x_, total_length);
      if (to_file_) {
        const auto filename = file_prefix_ + "lap_" + std::to_string(lap_count_);
        std::cout << "Saving lap to " << filename << std::endl;
        last_x_.T().to_file(filename + "_x.txt", "txt");
        last_u_.T().to_file(filename + "_u.txt", "txt");
        last_t_.T().to_file(filename + "_t.txt", "txt");
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
  } else {
    last_x_ = casadi::DM::horzcat({last_x_, x});
    last_u_ = casadi::DM::horzcat({last_u_, u});
    last_t_ = casadi::DM::horzcat({last_t_, t});
  }
}

}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
