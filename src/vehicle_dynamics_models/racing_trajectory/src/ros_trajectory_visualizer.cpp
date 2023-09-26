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

#include "racing_trajectory/ros_trajectory_visualizer.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{

ROSTrajectoryVisualizer::ROSTrajectoryVisualizer(RacingTrajectory & trajectory)
{
  const auto abscissa =
    casadi::DM::linspace(0.0, trajectory.total_length(), 1001)(casadi::Slice(0, -1));
  const auto N = abscissa.size1();

  abscissa_polygon_msg_ = std::make_shared<PolygonStamped>();
  abscissa_polygon_msg_->header.frame_id = "map";
  const auto pts = casadi::DM::horzcat(
        {
          trajectory.x_interpolation_function()(abscissa)[0],
          trajectory.y_interpolation_function()(abscissa)[0]
        }).T();
  abscissa_polygon_msg_->polygon = build_polygon(pts);

  left_boundary_polygon_msg_ = std::make_shared<PolygonStamped>();
  left_boundary_polygon_msg_->header.frame_id = "map";
  right_boundary_polygon_msg_ = std::make_shared<PolygonStamped>();
  right_boundary_polygon_msg_->header.frame_id = "map";
  const auto & left_boundary_frenet = casadi::DM::horzcat(
        {
          abscissa,
          trajectory.left_boundary_interpolation_function()(abscissa)[0],
          casadi::DM::zeros(N)
        }).T();
  const auto & right_boundary_frenet = casadi::DM::horzcat(
        {
          abscissa,
          trajectory.right_boundary_interpolation_function()(abscissa)[0],
          casadi::DM::zeros(N)
        }).T();
  const auto left_boundary_global =
    trajectory.frenet_to_global_function().map(N)(left_boundary_frenet)[0];
  const auto right_boundary_global =
    trajectory.frenet_to_global_function().map(N)(right_boundary_frenet)[0];
  left_boundary_polygon_msg_->polygon = build_polygon(left_boundary_global);
  right_boundary_polygon_msg_->polygon = build_polygon(right_boundary_global);
}

ROSTrajectoryVisualizer::~ROSTrajectoryVisualizer()
{
  if (static_vis_timer_) {
    static_vis_timer_->cancel();
    static_vis_timer_.reset();
  }

  if (left_boundary_polygon_pub_) {
    left_boundary_polygon_pub_.reset();
  }

  if (right_boundary_polygon_pub_) {
    right_boundary_polygon_pub_.reset();
  }

  if (abscissa_polygon_pub_) {
    abscissa_polygon_pub_.reset();
  }
}

void ROSTrajectoryVisualizer::attach_ros_publishers(
  rclcpp::Node * node, const double & dt,
  const bool & vis_boundary,
  const bool & vis_abscissa)
{
  node_ = node;
  if (vis_boundary) {
    left_boundary_polygon_pub_ = node->create_publisher<PolygonStamped>(
      "left_boundary_polygon", 1);
    right_boundary_polygon_pub_ = node->create_publisher<PolygonStamped>(
      "right_boundary_polygon", 1);
  }
  if (vis_abscissa) {
    abscissa_polygon_pub_ = node->create_publisher<PolygonStamped>(
      "abscissa_polygon", 1);
  }

  vis_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  static_vis_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(dt * 1000.0)),
    std::bind(&ROSTrajectoryVisualizer::on_static_vis_timer, this), vis_callback_group_);
}

Polygon ROSTrajectoryVisualizer::build_polygon(const casadi::DM & pts)
{
  Polygon polygon;
  polygon.points.reserve(pts.size2());
  for (int i = 0; i < pts.size2(); ++i) {
    auto & pt = polygon.points.emplace_back();
    pt.x = static_cast<double>(pts(0, i));
    pt.y = static_cast<double>(pts(1, i));
  }
  return polygon;
}

void ROSTrajectoryVisualizer::on_static_vis_timer()
{
  const auto now = node_->now();
  if (abscissa_polygon_pub_) {
    abscissa_polygon_msg_->header.stamp = now;
    abscissa_polygon_pub_->publish(*abscissa_polygon_msg_);
  }
  if (left_boundary_polygon_pub_ && right_boundary_polygon_pub_) {
    left_boundary_polygon_msg_->header.stamp = now;
    left_boundary_polygon_pub_->publish(*left_boundary_polygon_msg_);
    right_boundary_polygon_msg_->header.stamp = now;
    right_boundary_polygon_pub_->publish(*right_boundary_polygon_msg_);
  }
}
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
