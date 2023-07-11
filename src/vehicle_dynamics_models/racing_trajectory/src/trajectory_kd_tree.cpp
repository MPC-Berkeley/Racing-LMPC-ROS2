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

#include "racing_trajectory/trajectory_kd_tree.hpp"

#include <CGAL/Line_2.h>
#include <CGAL/Point_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/squared_distance_2.h>

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
TrajectoryKDTree::TrajectoryKDTree(const std::vector<double> & x, const std::vector<double> & y)
: x_(x), y_(y)
{
  if (x.size() != y.size()) {
    throw std::invalid_argument("x and y must have the same size.");
  }
  map_.reserve(x.size());
  tree_.reserve(x.size());
  for (size_t i = 0; i < x.size(); i++) {
    map_.insert({WaypointKey{x[i], y[i]}, i});
    tree_.insert(Point_d(x[i], y[i]));
  }
  tree_.build();
}

size_t TrajectoryKDTree::find_closest_waypoint_index(
  const double & x, const double & y) const
{
  Point_d query(x, y);
  Neighbor_search search(tree_, query, 1);
  const auto & closest_pt = *search.begin();
  return map_.find(WaypointKey{closest_pt.first.x(), closest_pt.first.y()})->second;
}

void TrajectoryKDTree::find_closest_waypoint_indices(
  const double & x, const double & y, const size_t & n,
  std::vector<size_t> & indices) const
{
  Point_d query(x, y);
  Neighbor_search search(tree_, query, n);
  indices.reserve(search.end() - search.begin());
  for (const auto & pt : search) {
    indices.push_back(map_.find(WaypointKey{pt.first.x(), pt.first.y()})->second);
  }
}

void TrajectoryKDTree::get_waypoint(const size_t & index, double & x, double & y) const
{
  x = x_[index];
  y = y_[index];
}
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
