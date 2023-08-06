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

#ifndef RACING_MPC__RACING_MPC_NODE_HPP_
#define RACING_MPC__RACING_MPC_NODE_HPP_

#include <memory>
#include <mutex>

#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <mpclab_msgs/msg/vehicle_state_msg.hpp>
#include <mpclab_msgs/msg/vehicle_actuation_msg.hpp>
#include <lmpc_transform_helper/lmpc_transform_helper.hpp>
#include <racing_trajectory/racing_trajectory.hpp>
#include <lmpc_utils/cycle_profiler.hpp>

#include "racing_mpc/racing_mpc_config.hpp"
#include "racing_mpc/racing_mpc.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
using lmpc::vehicle_model::racing_trajectory::RacingTrajectory;
class RacingMPCNode : public rclcpp::Node
{
public:
  explicit RacingMPCNode(const rclcpp::NodeOptions & options);

protected:
  double dt_;
  RacingMPCConfig::SharedPtr config_ {};
  RacingTrajectory::SharedPtr track_ {};
  BaseVehicleModel::SharedPtr model_ {};
  RacingMPC::SharedPtr mpc_ {};
  RacingMPC::SharedPtr mpc_full_ {};
  lmpc::utils::CycleProfiler<double>::UniquePtr profiler_ {};
  lmpc::utils::CycleProfiler<double>::UniquePtr profiler_iter_count_ {};
  std::mutex state_msg_mutex_;

  casadi::DM last_x_;
  casadi::DM last_u_;
  casadi::DM last_du_;
  casadi::DMDict sol_in_;
  casadi::Function f2g_;
  casadi::Function discrete_dynamics_ {};

  mpclab_msgs::msg::VehicleStateMsg::SharedPtr vehicle_state_msg_ {};
  mpclab_msgs::msg::VehicleActuationMsg::SharedPtr vehicle_actuation_msg_ {};

  // publishers (to world/simulator)
  rclcpp::Publisher<mpclab_msgs::msg::VehicleActuationMsg>::SharedPtr vehicle_actuation_pub_ {};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_vis_pub_ {};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_vis_pub_ {};

  // publishers (to diagnostics)
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_ {};

  // subscribers (from world/simulator)
  rclcpp::Subscription<mpclab_msgs::msg::VehicleStateMsg>::SharedPtr vehicle_state_sub_ {};

  // timers
  // republish vehicle state (TODO(haoru): to be replaced by a service)
  rclcpp::TimerBase::SharedPtr step_timer_;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;

  // callbacks
  void on_new_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg);
  void on_step_timer();
};
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_MPC__RACING_MPC_NODE_HPP_
