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

#ifndef VANILLA_CONTROLLER__VANILLA_CONTROLLER_NODE_HPP_
#define VANILLA_CONTROLLER__VANILLA_CONTROLLER_NODE_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>

#include <mpclab_msgs/msg/vehicle_state_msg.hpp>
#include <mpclab_msgs/msg/vehicle_actuation_msg.hpp>
#include <lmpc_transform_helper/lmpc_transform_helper.hpp>
#include <racing_trajectory/racing_trajectory.hpp>

#include "vanilla_controller/vanilla_controller_config.hpp"
#include "vanilla_controller/vanilla_controller.hpp"

namespace lmpc
{
namespace mpc
{
namespace vanilla_controller
{
using lmpc::vehicle_model::racing_trajectory::RacingTrajectory;
class VanillaControllerNode : public rclcpp::Node
{
public:
  explicit VanillaControllerNode(const rclcpp::NodeOptions & options);

protected:
  double dt_;
  VanillaControllerConfig::SharedPtr config_ {};
  RacingTrajectory::SharedPtr track_ {};
  BaseVehicleModel::SharedPtr model_ {};
  VanillaController::SharedPtr controller_ {};

  mpclab_msgs::msg::VehicleStateMsg::SharedPtr vehicle_state_msg_ {};
  mpclab_msgs::msg::VehicleActuationMsg::SharedPtr vehicle_actuation_msg_ {};

  // publishers (to world/simulator)
  rclcpp::Publisher<mpclab_msgs::msg::VehicleActuationMsg>::SharedPtr vehicle_actuation_pub_ {};

  // subscribers (from world/simulator)
  rclcpp::Subscription<mpclab_msgs::msg::VehicleStateMsg>::SharedPtr vehicle_state_sub_ {};

  // timers
  // republish vehicle state (TODO(haoru): to be replaced by a service)
  rclcpp::TimerBase::SharedPtr step_timer_;

  // callbacks
  void on_new_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg);
  void on_step_timer();
};
}  // namespace vanilla_controller
}  // namespace mpc
}  // namespace lmpc
#endif  // VANILLA_CONTROLLER__VANILLA_CONTROLLER_NODE_HPP_
