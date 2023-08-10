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

#ifndef RACING_TRAJECTORY__RACING_TRAJECTORY_MAP_HPP_
#define RACING_TRAJECTORY__RACING_TRAJECTORY_MAP_HPP_

#include <memory>
#include <string>
#include <map>

#include <casadi/casadi.hpp>

#include <racing_trajectory/racing_trajectory.hpp>

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
/**
 * @brief A set of racing trajectories.
 *
 */
class RacingTrajectoryMap
{
public:
  typedef std::shared_ptr<RacingTrajectoryMap> SharedPtr;
  typedef std::unique_ptr<RacingTrajectoryMap> UniquePtr;

  explicit RacingTrajectoryMap(const std::string & directory_path);

  RacingTrajectory::SharedPtr get_trajectory(const int & index);

private:
  std::map<int, RacingTrajectory::SharedPtr> trajectories_;
};
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // RACING_TRAJECTORY__RACING_TRAJECTORY_MAP_HPP_
