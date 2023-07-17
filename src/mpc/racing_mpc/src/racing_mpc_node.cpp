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
#include <base_vehicle_model/ros_param_loader.hpp>
#include <single_track_planar_model/ros_param_loader.hpp>

#include "racing_mpc/racing_mpc_node.hpp"
#include "racing_mpc/ros_param_loader.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
RacingMPCNode::RacingMPCNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("racing_mpc_node", options),
  dt_(utils::declare_parameter<double>(this, "racing_mpc_node.dt")),
  config_(lmpc::mpc::racing_mpc::load_parameters(this)),
  track_(std::make_shared<RacingTrajectory>(
      utils::declare_parameter<std::string>(
        this, "racing_mpc_node.race_track_file_path"))),
  model_(std::make_shared<SingleTrackPlanarModel>(
      lmpc::vehicle_model::base_vehicle_model::
      load_parameters(this), lmpc::vehicle_model::single_track_planar_model::load_parameters(
        this))),
  mpc_(std::make_shared<RacingMPC>(config_, model_)),
  f2g_(track_->frenet_to_global_function().map(mpc_->get_config().N))
{
  // initialize the actuation message
  vehicle_actuation_msg_ = std::make_shared<mpclab_msgs::msg::VehicleActuationMsg>();

  // initialize the mpc inputs
  const auto N = static_cast<casadi_int>(mpc_->get_config().N);
  sol_in_["T_ref"] = casadi::DM::zeros(1, N - 1) + dt_;
  sol_in_["total_length"] = track_->total_length();

  // build discrete dynamics
  const auto x_sym = casadi::MX::sym("x", model_->nx());
  const auto u_sym = casadi::MX::sym("u", model_->nu());
  const auto k = track_->curvature_interpolation_function()(x_sym(XIndex::PX))[0];

  const auto out1 = model_->dynamics()(casadi::MXDict{{"x", x_sym}, {"u", u_sym}, {"k", k}});
  const auto k1 = out1.at("x_dot");
  const auto out2 = model_->dynamics()({{"x", x_sym + dt_ / 2.0 * k1}, {"u", u_sym}, {"k", k}});
  const auto k2 = out2.at("x_dot");
  const auto out3 = model_->dynamics()({{"x", x_sym + dt_ / 2.0 * k2}, {"u", u_sym}, {"k", k}});
  const auto k3 = out3.at("x_dot");
  const auto out4 = model_->dynamics()({{"x", x_sym + dt_ * k3}, {"u", u_sym}, {"k", k}});
  const auto k4 = out4.at("x_dot");
  const auto out = x_sym + dt_ / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  discrete_dynamics_ = casadi::Function("discrete_dynamics", {x_sym, u_sym}, {out});

  // initialize the publishers
  vehicle_actuation_pub_ = this->create_publisher<mpclab_msgs::msg::VehicleActuationMsg>(
    "vehicle_actuation", 1);
  mpc_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_visualization", 1);
  ref_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>("ref_visualization", 1);

  // initialize the subscribers
  vehicle_state_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleStateMsg>(
    "vehicle_state", 1, std::bind(&RacingMPCNode::on_new_state, this, std::placeholders::_1));

  // initialize the timers
  // step_timer_ = this->create_wall_timer(
  //   std::chrono::duration<double>(dt_), std::bind(&RacingMPCNode::on_step_timer, this));
}

void RacingMPCNode::on_new_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg)
{
  vehicle_state_msg_ = msg;
  on_step_timer();
}

void RacingMPCNode::on_step_timer()
{
  // return if no state message is received
  if (!vehicle_state_msg_) {
    return;
  }

  using casadi::DM;
  using casadi::Slice;
  const auto N = static_cast<casadi_int>(mpc_->get_config().N);

  // prepare the mpc inputs
  const auto & p = vehicle_state_msg_->p;
  const auto & pt = vehicle_state_msg_->pt;
  const auto & v = vehicle_state_msg_->v;
  const auto x_ic = DM{
    p.s, p.x_tran, p.e_psi, v.v_long, v.v_tran, pt.de_psi
  };
  sol_in_["x_ic"] = x_ic;
  // std::cout << "x_ic: " << x_ic << std::endl;

  // if the mpc is not solved, pass the initial guess
  if (!mpc_->solved()) {
    last_x_ = DM::zeros(mpc_->get_model().nx(), N);
    last_u_ = DM::zeros(mpc_->get_model().nu(), N - 1);
    last_x_(XIndex::PX, 0) = x_ic(XIndex::PX);
    const auto v0 = x_ic(XIndex::VX);
    last_x_(XIndex::VX, 0) = v0;
    for (int i = 1; i < N; i++) {
      last_x_(XIndex::PX, i) = last_x_(XIndex::PX, i - 1) + dt_ * v0;
      last_x_(XIndex::VX, i) = v0;
    }
    sol_in_["X_optm_ref"] = last_x_;
    sol_in_["U_optm_ref"] = last_u_;
    sol_in_["T_optm_ref"] = sol_in_.at("T_ref");
    sol_in_["X_ref"] = last_x_;
    sol_in_["U_ref"] = last_u_;
  }

  // prepare the reference trajectory
  const auto abscissa = last_x_(XIndex::PX, Slice());
  const auto left_ref = track_->left_boundary_interpolation_function()(abscissa)[0];
  const auto right_ref = track_->right_boundary_interpolation_function()(abscissa)[0];
  const auto curvature_ref = track_->curvature_interpolation_function()(abscissa)[0];
  const auto vel_ref = track_->velocity_interpolation_function()(abscissa)[0];
  sol_in_["bound_left"] = left_ref;
  sol_in_["bound_right"] = right_ref;
  sol_in_["curvatures"] = curvature_ref;
  sol_in_["vel_ref"] = vel_ref;

  // solve the mpc
  auto sol_out = casadi::DMDict{};
  const auto start = std::chrono::high_resolution_clock::now();
  mpc_->solve(sol_in_, sol_out);
  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "MPC Execution Time: " << duration.count() << "ms" << std::endl;
  // if (mpc_->solved()) {
  //   sol_in_.erase("X_optm_ref");
  //   sol_in_.erase("U_optm_ref");
  //   sol_in_.erase("T_optm_ref");
  // }

  last_x_ = sol_out["X_optm"];
  last_u_ = sol_out["U_optm"];
  auto last_x_global = last_x_;
  last_x_global(Slice(XIndex::PX, XIndex::YAW + 1), Slice()) =
    f2g_(last_x_(Slice(XIndex::PX, XIndex::YAW + 1), Slice()))[0];

  // publish the actuation message
  const auto now = this->now();
  const auto u_vec = last_u_(Slice(), 0).get_elements();
  // std::cout << "x: " << last_x_(Slice(), 0) << std::endl;
  // std::cout << "u: " << last_u_(Slice(), 0) << std::endl;
  // std::cout << "xip1: "
  //   << discrete_dynamics_(casadi::DMVector{last_x_(Slice(), 0), last_u_(Slice(), 0)})[0]
  //   << std::endl << std::endl;
  vehicle_actuation_msg_->header.stamp = now;
  if (abs(u_vec[UIndex::FD]) > abs(u_vec[UIndex::FB])) {
    vehicle_actuation_msg_->u_a = u_vec[UIndex::FD];
  } else {
    vehicle_actuation_msg_->u_a = u_vec[UIndex::FB];
  }
  vehicle_actuation_msg_->u_steer = u_vec[UIndex::STEER];
  vehicle_actuation_pub_->publish(*vehicle_actuation_msg_);

  // publish the visualization message
  auto mpc_vis_msg = nav_msgs::msg::Path();
  mpc_vis_msg.header.stamp = now;
  mpc_vis_msg.header.frame_id = "map";
  mpc_vis_msg.poses.reserve(N);
  for (int i = 0; i < N; i++) {
    auto & pose = mpc_vis_msg.poses.emplace_back();
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = last_x_global(XIndex::PX, i).get_elements()[0];
    pose.pose.position.y = last_x_global(XIndex::PY, i).get_elements()[0];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(
      utils::TransformHelper::quaternion_from_heading(
        last_x_global(XIndex::YAW, i).get_elements()[0]));
  }
  mpc_vis_pub_->publish(mpc_vis_msg);
  // std::cout << last_x_ << std::endl;

  // prepare the next reference
  last_x_ = DM::horzcat({last_x_(Slice(), Slice(1, N)), DM::zeros(model_->nx(), 1)});
  last_u_ = DM::horzcat({last_u_(Slice(), Slice(1, N - 1)), last_u_(Slice(), Slice(N - 2))});
  last_x_(Slice(), -1) =
    discrete_dynamics_(casadi::DMVector{last_x_(Slice(), -2), last_u_(Slice(), -1)})[0];
  sol_in_["X_ref"] = last_x_;
  sol_in_["U_ref"] = last_u_;
  sol_in_["X_optm_ref"] = last_x_;
  sol_in_["U_optm_ref"] = last_u_;
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lmpc::mpc::racing_mpc::RacingMPCNode)
