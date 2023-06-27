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

#ifndef RACING_TRAJECTORY__TRAJECTORY_KD_TREE_HPP_
#define RACING_TRAJECTORY__TRAJECTORY_KD_TREE_HPP_

#include <vector>
#include <unordered_map>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_d;
typedef CGAL::Search_traits_2<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
/**
 * @brief Internal map key to map waypoints to their indices.
 *
 */
struct WaypointKey
{
  double x;
  double y;
  bool operator==(const WaypointKey & other) const
  {
    return x == other.x && y == other.y;
  }
};

/**
 * @brief Hasher for WaypointKey for interal use.
 *
 */
struct WaypointKeyHasher
{
  std::size_t operator()(const WaypointKey & k) const
  {
    return std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1);
  }
};

/**
 * @brief KDTree for fast lookup of closest waypoints.
 *
 */
class TrajectoryKDTree
{
public:
  /**
   * @brief Construct a new TrajectoryKDTree object.
   *
   * @param x the x coordinates of the waypoints.
   * @param y the y coordinates of the waypoints.
   */
  TrajectoryKDTree(const std::vector<double> & x, const std::vector<double> & y);

  /**
   * @brief Find the closest waypoint to a given point.
   *
   * @param x the x coordinate of the point.
   * @param y the y coordinate of the point.
   * @return size_t the index of the closest waypoint.
   */
  size_t find_closest_waypoint_index(
    const double & x, const double & y
  ) const;

  /**
   * @brief Find the n closest waypoints to a given point.
   *
   * @param x the x coordinate of the point.
   * @param y the y coordinate of the point.
   * @param n the number of waypoints to find.
   * @param indices the indices of the closest waypoints.
   */
  void find_closest_waypoint_indices(
    const double & x, const double & y, const size_t & n, std::vector<size_t> & indices
  ) const;

  /**
   * @brief Get the waypoint.
   *
   * @param index the index of the waypoint.
   * @param x the x coordinate of the waypoint.
   * @param y the y coordinate of the waypoint.
   */
  void get_waypoint(
    const size_t & index, double & x, double & y
  ) const;

protected:
  typedef std::unordered_map<WaypointKey, size_t, WaypointKeyHasher> WaypointMap;

  WaypointMap map_;
  Tree tree_;
  std::vector<double> x_;
  std::vector<double> y_;
};
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // RACING_TRAJECTORY__TRAJECTORY_KD_TREE_HPP_
