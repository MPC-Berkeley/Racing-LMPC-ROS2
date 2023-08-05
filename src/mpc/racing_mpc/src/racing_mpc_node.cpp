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
  model_(vehicle_model::vehicle_model_factory::load_vehicle_model(
      utils::declare_parameter<std::string>(
        this, "racing_mpc_node.vehicle_model_name"), this)),
  mpc_(std::make_shared<RacingMPC>(config_, model_, true)),
  profiler_(std::make_unique<lmpc::utils::CycleProfiler<double>>(10)),
  profiler_iter_count_(std::make_unique<lmpc::utils::CycleProfiler<double>>(10)),
  f2g_(track_->frenet_to_global_function().map(mpc_->get_config().N))
{
  auto full_config = std::make_shared<RacingMPCConfig>(*config_);
  full_config->max_cpu_time = 10.0;
  full_config->max_iter = 1000;
  mpc_full_ = std::make_shared<RacingMPC>(full_config, model_, true);

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
  const auto xip1 = model_->discrete_dynamics()(
    casadi::MXDict{{"x", x_sym}, {"u", u_sym}, {"k", k}, {"dt", dt_}}
  ).at("xip1");
  discrete_dynamics_ = casadi::Function("discrete_dynamics", {x_sym, u_sym}, {xip1});

  // initialize the publishers
  vehicle_actuation_pub_ = this->create_publisher<mpclab_msgs::msg::VehicleActuationMsg>(
    "vehicle_actuation", 1);
  mpc_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_visualization", 1);
  ref_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>("ref_visualization", 1);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 1);

  // initialize the subscribers
  // state subscription is on a separate callback group
  // Together with the multi-threaded executor (defined in main()), this allows the state
  // subscription to be processed in parallel with the mpc computation in continuous mode
  state_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = state_callback_group_;
  vehicle_state_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleStateMsg>(
    "vehicle_state", 1, std::bind(
      &RacingMPCNode::on_new_state, this,
      std::placeholders::_1), sub_options);

  if (config_->step_mode == RacingMPCStepMode::CONTINUOUS) {
    // initialize the timers
    step_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&RacingMPCNode::on_step_timer, this));
  }
}

void RacingMPCNode::on_new_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg)
{
  state_msg_mutex_.lock();
  vehicle_state_msg_ = msg;
  state_msg_mutex_.unlock();
  if (config_->step_mode == RacingMPCStepMode::STEP) {
    on_step_timer();
  }
}

void RacingMPCNode::on_step_timer()
{
  using casadi::DM;
  using casadi::Slice;
  static bool jitted = !config_->jit;  // if JIT is done

  // return if no state message is received
  state_msg_mutex_.lock();
  if (!vehicle_state_msg_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for first vehicle state message.");
    state_msg_mutex_.unlock();
    return;
  }
  const auto & p = vehicle_state_msg_->p;
  const auto & v = vehicle_state_msg_->v;
  const auto & w = vehicle_state_msg_->w;
  const auto x_ic_base = DM{
    p.s, p.x_tran, p.e_psi, v.v_long, v.v_tran, w.w_psi
  };
  state_msg_mutex_.unlock();

  const auto mpc_solve_start = std::chrono::system_clock::now();
  static size_t profile_step_count = 0;
  const auto N = static_cast<casadi_int>(mpc_->get_config().N);

  // prepare the mpc inputs
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
  // current input
  sol_in_["u_ic"] = u_ic;

  // std::cout << "x_ic: " << x_ic << std::endl;

  // if the mpc is not solved, pass the initial guess
  if (!mpc_full_->solved()) {
    last_x_ = DM::zeros(mpc_->get_model().nx(), N);
    last_u_ = DM::zeros(mpc_->get_model().nu(), N - 1) + 1e-9;
    last_x_(Slice(), 0) = x_ic;
    const auto v0 = x_ic_base(XIndex::VX);
    for (int i = 1; i < N; i++) {
      last_x_(
        Slice(),
        i) =
        discrete_dynamics_(
        casadi::DMVector{last_x_(Slice(), i - 1),
          last_u_(Slice(), i - 1)})[0];
    }
    sol_in_["X_optm_ref"] = last_x_;
    sol_in_["U_optm_ref"] = last_u_;
    sol_in_["T_optm_ref"] = sol_in_.at("T_ref");
    sol_in_["X_ref"] = last_x_;
    sol_in_["U_ref"] = last_u_;
    sol_in_["x_ic"] = x_ic;
  } else {
    // prepare the next reference
    if (config_->step_mode == RacingMPCStepMode::CONTINUOUS) {
      sol_in_["x_ic"] = discrete_dynamics_(casadi::DMVector{x_ic, last_u_(Slice(), 0)})[0];
    } else if (config_->step_mode == RacingMPCStepMode::STEP) {
      sol_in_["x_ic"] = x_ic;
    } else {
      throw std::runtime_error("Unknown RacingMPCStepMode");
    }
    last_x_ = DM::horzcat({last_x_(Slice(), Slice(1, N)), DM::zeros(model_->nx(), 1)});
    last_u_ = DM::horzcat({last_u_(Slice(), Slice(1, N - 1)), last_u_(Slice(), Slice(N - 2))});
    last_x_(Slice(), -1) =
      discrete_dynamics_(casadi::DMVector{last_x_(Slice(), -2), last_u_(Slice(), -1)})[0];
    sol_in_["X_ref"] = last_x_;
    sol_in_["U_ref"] = last_u_;
    sol_in_["X_optm_ref"] = last_x_;
    sol_in_["U_optm_ref"] = last_u_;
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
  auto stats = casadi::Dict{};

  // solve the first time with full dynamics
  if (!mpc_full_->solved()) {
    RCLCPP_INFO(this->get_logger(), "Get initial solution with full dynamics.");
    mpc_full_->solve(sol_in_, sol_out, stats);
    last_x_ = sol_out["X_optm"];
    last_u_ = sol_out["U_optm"];
    if (mpc_full_->solved()) {
      RCLCPP_INFO(this->get_logger(), "Solved the first time with full dynamics.");
    } else {
      RCLCPP_FATAL(this->get_logger(), "Failed to solve the first time with full dynamics.");
    }
    return;
  }

  // const auto mpc_start = std::chrono::high_resolution_clock::now();
  if (!jitted) {
    RCLCPP_INFO(this->get_logger(), "Using the first solve to execute just-in-time compilation.");
  }
  mpc_->solve(sol_in_, sol_out, stats);
  // const auto mpc_stop = std::chrono::high_resolution_clock::now();
  // const auto mpc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
  //   mpc_stop - mpc_start);
  // std::cout << "MPC Execution Time: " << mpc_duration.count() << "ms" << std::endl;
  // if (mpc_->solved()) {
  //   sol_in_.erase("X_optm_ref");
  //   sol_in_.erase("U_optm_ref");
  //   sol_in_.erase("T_optm_ref");
  // }

  last_x_ = sol_out["X_optm"];
  last_u_ = sol_out["U_optm"];

  if (!jitted) {
    // on first solve, exit since JIT will take a long time
    jitted = true;
    RCLCPP_INFO(this->get_logger(), "JIT is done. Discarding the first solve...");
    return;
  }

  auto last_x_global = last_x_;
  last_x_global(Slice(XIndex::PX, XIndex::YAW + 1), Slice()) =
    f2g_(last_x_(Slice(XIndex::PX, XIndex::YAW + 1), Slice()))[0];

  const auto mpc_solve_duration = std::chrono::system_clock::now() - mpc_solve_start;
  profiler_->add_cycle_stats(mpc_solve_duration.count() * 1e-6);
  profiler_iter_count_->add_cycle_stats(static_cast<double>(stats.at("iter_count")));
  profile_step_count++;

  // sleep if the execution time is less than dt
  if (config_->step_mode == RacingMPCStepMode::CONTINUOUS) {
    if (mpc_solve_duration < std::chrono::duration<double>(dt_)) {
      rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(dt_) - mpc_solve_duration));
    }
  }

  // record the MPC publish time
  const auto now = this->now();

  if (profile_step_count % profiler_->capacity() == 0) {
    auto diagnostics_msg = diagnostic_msgs::msg::DiagnosticArray();
    diagnostics_msg.status.push_back(
      profiler_->profile().to_diagnostic_status(
        "Racing MPC Solve Time", "(ms)", dt_ * 1e3));
    diagnostics_msg.status.push_back(
      profiler_iter_count_->profile().to_diagnostic_status(
        "Racing MPC Iteration Count", "Number of Solver Iterations", 50));
    diagnostics_msg.header.stamp = now;
    diagnostics_pub_->publish(diagnostics_msg);
    profile_step_count = 0;
  }
  // publish the actuation message
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
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<lmpc::mpc::racing_mpc::RacingMPCNode>(options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
