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

#ifndef RACING_TRAJECTORY__ROS_TRAJECTORY_VISUALIZER_HPP_
#define RACING_TRAJECTORY__ROS_TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>
#include <shared_mutex>

#include <casadi/casadi.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "racing_trajectory/racing_trajectory.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::PolygonStamped;

class ROSTrajectoryVisualizer
{
public:
  typedef std::shared_ptr<ROSTrajectoryVisualizer> SharedPtr;
  typedef std::unique_ptr<ROSTrajectoryVisualizer> UniquePtr;

  explicit ROSTrajectoryVisualizer(RacingTrajectory & trajectory);
  ~ROSTrajectoryVisualizer();

  void change_trajectory(RacingTrajectory & trajectory);

  void attach_ros_publishers(
    rclcpp::Node * node, const double & dt, const bool & vis_boundary,
    const bool & vis_abscissa);

private:
  PolygonStamped::SharedPtr left_boundary_polygon_msg_ {};
  PolygonStamped::SharedPtr right_boundary_polygon_msg_ {};
  PolygonStamped::SharedPtr abscissa_polygon_msg_ {};

  rclcpp::Publisher<PolygonStamped>::SharedPtr left_boundary_polygon_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr right_boundary_polygon_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr abscissa_polygon_pub_;

  rclcpp::TimerBase::SharedPtr static_vis_timer_;
  rclcpp::CallbackGroup::SharedPtr vis_callback_group_;

  rclcpp::Node * node_;

  std::shared_mutex mutex_;

  Polygon build_polygon(const casadi::DM & pts);
  void on_static_vis_timer();
};
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // RACING_TRAJECTORY__ROS_TRAJECTORY_VISUALIZER_HPP_
