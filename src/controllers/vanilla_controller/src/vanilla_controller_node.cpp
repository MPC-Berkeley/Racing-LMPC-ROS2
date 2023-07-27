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

#include <lmpc_utils/ros_param_helper.hpp>
#include <lmpc_utils/utils.hpp>
#include <base_vehicle_model/ros_param_loader.hpp>
#include <single_track_planar_model/ros_param_loader.hpp>

#include "vanilla_controller/vanilla_controller_node.hpp"
#include "vanilla_controller/ros_param_loader.hpp"

namespace lmpc
{
namespace mpc
{
namespace vanilla_controller
{
VanillaControllerNode::VanillaControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vanilla_controller_node", options),
  dt_(utils::declare_parameter<double>(this, "vanilla_controller_node.dt")),
  config_(lmpc::mpc::vanilla_controller::load_parameters(this)),
  track_(std::make_shared<RacingTrajectory>(
      utils::declare_parameter<std::string>(
        this, "vanilla_controller_node.race_track_file_path"))),
  model_(vehicle_model::vehicle_model_factory::load_vehicle_model(
      utils::declare_parameter<std::string>(
        this, "vanilla_controller_node.vehicle_model_name"), this)),
  controller_(std::make_shared<VanillaController>(config_, model_, track_))
{
  // initialize the actuation message
  vehicle_actuation_msg_ = std::make_shared<mpclab_msgs::msg::VehicleActuationMsg>();

  // initialize the publishers
  vehicle_actuation_pub_ = this->create_publisher<mpclab_msgs::msg::VehicleActuationMsg>(
    "vehicle_actuation", 1);

  // initialize the subscribers
  vehicle_state_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleStateMsg>(
    "vehicle_state", 1,
    std::bind(&VanillaControllerNode::on_new_state, this, std::placeholders::_1));

  if (config_->step_mode == VanillaControllerStepMode::CONTINUOUS) {
    // initialize the timers
    step_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&VanillaControllerNode::on_step_timer, this));
  }
}

void VanillaControllerNode::on_new_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg)
{
  vehicle_state_msg_ = msg;
  if (config_->step_mode == VanillaControllerStepMode::STEP) {
    on_step_timer();
  }
}

void VanillaControllerNode::on_step_timer()
{
  // return if no state message is received
  if (!vehicle_state_msg_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for first vehicle state message.");
    return;
  }
  auto sol_in = casadi::DMDict{};
  using casadi::DM;
  using casadi::Slice;

  // prepare the mpc inputs
  const auto & p = vehicle_state_msg_->p;
  const auto & v = vehicle_state_msg_->v;
  const auto & w = vehicle_state_msg_->w;
  const auto x_ic_base = DM{
    p.s, p.x_tran, p.e_psi, v.v_long, v.v_tran, w.w_psi
  };
  const auto u_ic_base = casadi::DM {
    vehicle_actuation_msg_->u_a > 0.0 ? vehicle_actuation_msg_->u_a : 0.0,
    vehicle_actuation_msg_->u_a < 0.0 ? vehicle_actuation_msg_->u_a : 0.0,
    vehicle_actuation_msg_->u_steer
  };
  const auto x_ic =
    model_->from_base_state()(casadi::DMDict{{"x", x_ic_base}, {"u", u_ic_base}}).at("x_out");
  const auto u_ic =
    model_->from_base_control()(
    casadi::DMDict{{"x", x_ic_base},
      {"u", u_ic_base}}).at("u_out");

  sol_in["u_ic"] = u_ic;
  sol_in["x_ic"] = x_ic;

  const auto vel_ref = track_->velocity_interpolation_function()(x_ic(XIndex::PX))[0];
  // const auto vel_ref = 8.0;
  sol_in["vel_ref"] = vel_ref;

  // solve the mpc
  auto sol_out = casadi::DMDict{};
  auto stats = casadi::Dict{};
  // const auto mpc_start = std::chrono::high_resolution_clock::now();
  controller_->solve(sol_in, sol_out, stats);

  // publish the actuation message
  const auto u_vec = sol_out.at("u_out").get_elements();
  const auto now = this->now();
  vehicle_actuation_msg_->header.stamp = now;
  if (abs(u_vec[UIndex::FD]) > abs(u_vec[UIndex::FB])) {
    vehicle_actuation_msg_->u_a = u_vec[UIndex::FD];
  } else {
    vehicle_actuation_msg_->u_a = u_vec[UIndex::FB];
  }
  vehicle_actuation_msg_->u_steer = u_vec[UIndex::STEER];
  vehicle_actuation_pub_->publish(*vehicle_actuation_msg_);
}
}  // namespace vanilla_controller
}  // namespace mpc
}  // namespace lmpc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<lmpc::mpc::vanilla_controller::VanillaControllerNode>(options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
