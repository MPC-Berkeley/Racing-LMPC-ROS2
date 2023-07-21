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

#ifndef RACING_SIMULATOR__RACING_SIMULATOR_NODE_HPP_
#define RACING_SIMULATOR__RACING_SIMULATOR_NODE_HPP_

#include <memory>
#include <vector>

#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mpclab_msgs/msg/vehicle_state_msg.hpp>
#include <mpclab_msgs/msg/vehicle_actuation_msg.hpp>
#include <lmpc_transform_helper/lmpc_transform_helper.hpp>

#include "racing_simulator/racing_simulator_config.hpp"
#include "racing_simulator/racing_simulator.hpp"

namespace lmpc
{
namespace simulation
{
namespace racing_simulator
{
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Polygon;
using nav_msgs::msg::Odometry;
class RacingSimulatorNode : public rclcpp::Node
{
public:
  explicit RacingSimulatorNode(const rclcpp::NodeOptions & options);

protected:
  RacingSimulatorConfig::SharedPtr config_ {};
  RacingTrajectory::SharedPtr track_ {};
  SingleTrackPlanarModel::SharedPtr model_ {};
  RacingSimulator::SharedPtr simulator_ {};
  uint64_t sim_step_ {0};
  uint64_t lap_count_ {0};
  tf2::Transform cg_to_baselink_ {};

  utils::TransformHelper tf_helper_;

  PolygonStamped::SharedPtr vehicle_polygon_msg_ {};
  PolygonStamped::SharedPtr left_boundary_polygon_msg_ {};
  PolygonStamped::SharedPtr right_boundary_polygon_msg_ {};
  PolygonStamped::SharedPtr abscissa_polygon_msg_ {};
  mpclab_msgs::msg::VehicleStateMsg::SharedPtr vehicle_state_msg_ {};
  mpclab_msgs::msg::VehicleActuationMsg::SharedPtr vehicle_actuation_msg_ {};
  TransformStamped::SharedPtr map_to_baselink_msg_ {};

  // publishers (to controller)
  rclcpp::Publisher<mpclab_msgs::msg::VehicleStateMsg>::SharedPtr vehicle_state_pub_;

  // publishers (to visualization)
  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_polygon_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr left_boundary_polygon_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr right_boundary_polygon_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr abscissa_polygon_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_pub_;

  // subscribers (from controller)
  rclcpp::Subscription<mpclab_msgs::msg::VehicleActuationMsg>::SharedPtr vehicle_actuation_sub_;

  // subscribers (reset state)
  rclcpp::Subscription<mpclab_msgs::msg::VehicleStateMsg>::SharedPtr reset_state_sub_;

  // timers
  // a slow rate timer to visualize track boundaries and abscissa
  rclcpp::TimerBase::SharedPtr static_vis_timer_;
  // step mode: republish vehicle state (TODO(haoru): to be replaced by a service)
  rclcpp::TimerBase::SharedPtr state_repub_timer_;
  // continuous mode: stream vehicle state
  rclcpp::TimerBase::SharedPtr sim_step_timer_;

  // callbacks
  void on_actuation(const mpclab_msgs::msg::VehicleActuationMsg::SharedPtr msg);
  void on_reset_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg);
  void on_static_vis_timer();
  void on_state_repub_timer();
  void on_state_update();

  // helper functions
  Polygon build_polygon(const casadi::DM & pts);
  void update_vehicle_state_msg(
    const std::vector<double> & x, const FrenetPose2D & frenet_pose,
    const Pose2D & global_pose);
};
}  // namespace racing_simulator
}  // namespace simulation
}  // namespace lmpc
#endif  // RACING_SIMULATOR__RACING_SIMULATOR_NODE_HPP_
