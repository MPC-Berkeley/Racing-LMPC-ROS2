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
  tracks_(std::make_shared<RacingTrajectoryMap>(
    utils::declare_parameter<std::string>(
        this, "racing_mpc_node.traj_folder")
  )),
  traj_idx_(utils::declare_parameter<int>(
        this, "racing_mpc_node.default_traj_idx")),
  delay_step_(utils::declare_parameter<int>(
        this, "racing_mpc_node.delay_step")),
  track_(tracks_->get_trajectory(traj_idx_)),
  vis_(std::make_unique<ROSTrajectoryVisualizer>(*track_)),
  model_(vehicle_model::vehicle_model_factory::load_vehicle_model(
      utils::declare_parameter<std::string>(
        this, "racing_mpc_node.vehicle_model_name"), this)),
  mpc_(std::make_shared<RacingMPC>(config_, model_, false)),
  profiler_(std::make_unique<lmpc::utils::CycleProfiler<double>>(10)),
  profiler_iter_count_(std::make_unique<lmpc::utils::CycleProfiler<double>>(10)),
  speed_scale_(utils::declare_parameter<double>(this, "racing_mpc_node.velocity_profile_scale")),
  f2g_(track_->frenet_to_global_function().map(mpc_->get_config().N))
{
  // add a full dynamics MPC solver for the problem initialization
  auto full_config = std::make_shared<RacingMPCConfig>(*config_);
  full_config->max_cpu_time = 10.0;
  full_config->max_iter = 1000;
  mpc_full_ = std::make_shared<RacingMPC>(full_config, model_, true);

  // add visualizations for the trajectory
  vis_->attach_ros_publishers(this, 1.0, true, true);

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
  ss_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ss_visualization", 1);
  mpc_telemetry_pub_ = this->create_publisher<lmpc_msgs::msg::MPCTelemetry>("mpc_telemetry", 1);

  // initialize the subscribers
  // state subscription is on a separate callback group
  // Together with the multi-threaded executor (defined in main()), this allows the state
  // subscription to be processed in parallel with the mpc computation in continuous mode
  state_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions state_sub_options;
  state_sub_options.callback_group = state_callback_group_;
  vehicle_state_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleStateMsg>(
    "vehicle_state", 1, std::bind(
      &RacingMPCNode::on_new_state, this,
      std::placeholders::_1), state_sub_options);

  trajectory_command_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions trajectory_command_sub_options;
  trajectory_command_sub_options.callback_group = trajectory_command_callback_group_;
  trajectory_command_sub_ = this->create_subscription<lmpc_msgs::msg::TrajectoryCommand>(
    "lmpc_trajectory_command", 1, std::bind(
      &RacingMPCNode::on_new_trajectory_command, this,
      std::placeholders::_1), trajectory_command_sub_options);

  // initialize the parameter callback
  callback_handle_ = add_on_set_parameters_callback(
    std::bind(&RacingMPCNode::on_set_parameters, this, std::placeholders::_1));

  if (config_->step_mode == RacingMPCStepMode::CONTINUOUS) {
    // initialize the timers
    step_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&RacingMPCNode::on_step_timer, this));
  }
}

void RacingMPCNode::on_new_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(state_msg_mutex_);
  vehicle_state_msg_ = msg;
  lock.unlock();
  if (config_->step_mode == RacingMPCStepMode::STEP) {
    on_step_timer();
  }
}

void RacingMPCNode::on_new_trajectory_command(
  const lmpc_msgs::msg::TrajectoryCommand::SharedPtr msg)
{
  std::shared_lock<std::shared_mutex> speed_limit_lock(speed_limit_mutex_);
  const auto current_speed_limit = speed_limit_;
  speed_limit_lock.unlock();
  if (current_speed_limit != msg->speed_limit) {
    set_speed_limit(msg->speed_limit);
  }

  std::shared_lock<std::shared_mutex> speed_scale_lock(speed_scale_mutex_);
  const auto current_speed_scale = speed_scale_;
  speed_scale_lock.unlock();
  if (current_speed_scale != msg->velocity_profile_scale) {
    set_speed_scale(msg->velocity_profile_scale);
  }

  std::shared_lock<std::shared_mutex> traj_lock(traj_mutex_);
  const auto current_traj_idx = traj_idx_;
  traj_lock.unlock();
  if (current_traj_idx != msg->trajectory_index) {
    change_trajectory(msg->trajectory_index);
  }
}

void RacingMPCNode::on_step_timer()
{
  using casadi::DM;
  using casadi::Slice;
  static bool jitted = !config_->jit;  // if JIT is done

  lmpc_msgs::msg::MPCTelemetry telemetry_msg;

  std::unique_lock<std::shared_mutex> traj_lock(traj_mutex_);
  telemetry_msg.trajectory_index = traj_idx_;
  // return if no state message is received
  std::shared_lock<std::shared_mutex> state_msg_lock(state_msg_mutex_);
  if (!vehicle_state_msg_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for first vehicle state message.");
    state_msg_lock.unlock();
    return;
  }
  const auto & p = vehicle_state_msg_->p;
  const auto & v = vehicle_state_msg_->v;
  const auto & w = vehicle_state_msg_->w;
  auto x_ic_base = DM{
    p.s, p.x_tran, p.e_psi, v.v_long, v.v_tran, w.w_psi
  };
  const Pose2D current_global_pose{{vehicle_state_msg_->x.x, vehicle_state_msg_->x.y}, vehicle_state_msg_->e.psi};
  state_msg_lock.unlock();

  const auto mpc_solve_start = std::chrono::system_clock::now();

  FrenetPose2D current_frenet_pose;
  track_->global_to_frenet(current_global_pose, current_frenet_pose);
  x_ic_base(XIndex::PX) = current_frenet_pose.position.s;
  x_ic_base(XIndex::PY) = current_frenet_pose.position.t;
  x_ic_base(XIndex::YAW) = current_frenet_pose.yaw;

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
  // current time
  sol_in_["t_ic"] = vehicle_state_msg_->t;

  // std::cout << "x_ic: " << x_ic << std::endl;

  // if the mpc is not solved, pass the initial guess
  if (!mpc_full_->solved()) {
    last_x_ = DM::zeros(mpc_->get_model().nx(), N);
    last_u_ = DM::zeros(mpc_->get_model().nu(), N - 1) + 1e-9;
    last_du_ = DM::zeros(mpc_->get_model().nu(), N - 1);
    if (config_->learning) {
      last_convex_combi_ = DM::zeros(config_->num_ss_pts);
    }
    last_x_(Slice(), 0) = x_ic;
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
    sol_in_["dU_optm_ref"] = last_du_;
    if (config_->learning) {
      sol_in_["convex_combi_optm_ref"] = last_convex_combi_;
    }
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
    last_du_ = DM::horzcat({last_du_(Slice(), Slice(1, N - 1)), DM::zeros(model_->nu(), 1)});
    last_x_(Slice(), -1) =
      discrete_dynamics_(casadi::DMVector{last_x_(Slice(), -2), last_u_(Slice(), -1)})[0];
    sol_in_["X_ref"] = last_x_;
    sol_in_["U_ref"] = last_u_;
    sol_in_["X_optm_ref"] = last_x_;
    sol_in_["U_optm_ref"] = last_u_;
    sol_in_["dU_optm_ref"] = last_du_;
    if (config_->learning) {
      sol_in_["convex_combi_ref"] = last_convex_combi_;
    }
  }

  // prepare the reference trajectory
  const auto abscissa = last_x_(XIndex::PX, Slice());
  const auto left_ref = track_->left_boundary_interpolation_function()(abscissa)[0];
  const auto right_ref = track_->right_boundary_interpolation_function()(abscissa)[0];
  const auto curvature_ref = track_->curvature_interpolation_function()(abscissa)[0];
  auto vel_ref = track_->velocity_interpolation_function()(abscissa)[0];
  // cap the velocity by the speed limit
  std::shared_lock<std::shared_mutex> speed_limit_lock(speed_limit_mutex_);
  std::shared_lock<std::shared_mutex> speed_scale_lock(speed_scale_mutex_);
  for (int i = 0; i < config_->N; i++) {
    // clip the velocity reference within +- 20m/s of current speed
    const auto current_speed = static_cast<double>(last_x_(XIndex::VX, i));
    const auto ref_speed = static_cast<double>(vel_ref(i)) * speed_scale_;
    const auto speed_limit_clipped = std::clamp(
      this->speed_limit_, current_speed - config_->max_vel_ref_diff, current_speed + config_->max_vel_ref_diff);
    // valid TTL has positive velocity profile.
    // if the velocity profile is negative, set it to the speed limit
    if (ref_speed > 0.0)
    {
      const auto ref_speed_clipped = std::clamp(
        ref_speed, current_speed - config_->max_vel_ref_diff, current_speed + config_->max_vel_ref_diff);
      vel_ref(i) = std::min(ref_speed_clipped, speed_limit_clipped);
    } else {
      vel_ref(i) = speed_limit_clipped;
    }
  }
  speed_limit_lock.unlock();
  speed_scale_lock.unlock();
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
    last_du_ = sol_out["dU_optm"];
    if (config_->learning) {
      last_convex_combi_ = sol_out["convex_combi_optm"];
    }
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

  if (sol_out.count("X_optm")) {
    last_x_ = sol_out["X_optm"];
    last_u_ = sol_out["U_optm"];
    last_du_ = sol_out["dU_optm"];
    telemetry_msg.solved = true;
  } else {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MPC could not be solved.");
    telemetry_msg.solved = false;
  }
  telemetry_msg.state = last_x_.get_elements();
  telemetry_msg.control = last_u_.get_elements();

  if (!jitted) {
    // on first solve, exit since JIT will take a long time
    jitted = true;
    RCLCPP_INFO(this->get_logger(), "JIT is done. Discarding the first solve...");
    return;
  }

  auto last_x_global = last_x_;
  last_x_global(Slice(XIndex::PX, XIndex::YAW + 1), Slice()) =
    f2g_(last_x_(Slice(XIndex::PX, XIndex::YAW + 1), Slice()))[0];

  auto x_ref_global = sol_in_.at("X_optm_ref");
  x_ref_global(Slice(XIndex::PX, XIndex::YAW + 1), Slice()) =
    f2g_(sol_in_.at("X_optm_ref")(Slice(XIndex::PX, XIndex::YAW + 1), Slice()))[0]; 

  const auto mpc_solve_duration = std::chrono::system_clock::now() - mpc_solve_start;
  const auto mpc_solve_duration_ms = mpc_solve_duration.count() * 1e-6;
  profiler_->add_cycle_stats(mpc_solve_duration_ms);
  telemetry_msg.solve_time = mpc_solve_duration_ms;
  if (stats.count("iter_count")) {
    profiler_iter_count_->add_cycle_stats(static_cast<double>(stats.at("iter_count")));
  }
  profile_step_count++;

  traj_lock.unlock();
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
  const auto last_u_base =
    model_->to_base_control()(
    casadi::DMDict{{"x", last_x_(Slice(), delay_step_)},
      {"u", last_u_(Slice(), delay_step_)}}).at("u_out");
  const auto u_vec = last_u_base.get_elements();
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
    pose.pose.position.x = x_ref_global(XIndex::PX, i).get_elements()[0];
    pose.pose.position.y = x_ref_global(XIndex::PY, i).get_elements()[0];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(
      utils::TransformHelper::quaternion_from_heading(
        x_ref_global(XIndex::YAW, i).get_elements()[0]));
  }
  mpc_vis_pub_->publish(mpc_vis_msg);

  // publish the ref visualization message
  auto ref_vis_msg = nav_msgs::msg::Path();
  ref_vis_msg.header.stamp = now;
  ref_vis_msg.header.frame_id = "map";
  ref_vis_msg.poses.reserve(N);
  for (int i = 0; i < N; i++) {
    auto & pose = ref_vis_msg.poses.emplace_back();
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = last_x_global(XIndex::PX, i).get_elements()[0];
    pose.pose.position.y = last_x_global(XIndex::PY, i).get_elements()[0];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(
      utils::TransformHelper::quaternion_from_heading(
        last_x_global(XIndex::YAW, i).get_elements()[0]));
  }
  ref_vis_pub_->publish(ref_vis_msg);

  // publish the safe set visualization message
  if (sol_out.count("ss_x"))
  {
    const auto & ss_X = sol_out["ss_x"];
    const auto & ss_J = sol_out["ss_j"];
    auto ss_vis_msg = visualization_msgs::msg::MarkerArray();
    auto & marker = ss_vis_msg.markers.emplace_back();
    marker.header.stamp = now;
    marker.header.frame_id = "map";
    marker.ns = "safe_set";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.points.reserve(ss_X.size2());
    for (casadi_int i = 0; i < ss_X.size2(); i++) {
      const auto ss_x_frenet = ss_X(Slice(XIndex::PX, XIndex::YAW + 1), i);
      const auto ss_x_global = track_->frenet_to_global_function()(ss_x_frenet)[0].get_elements();
      auto & point = marker.points.emplace_back();
      point.x = ss_x_global[0];
      point.y = ss_x_global[1];
      point.z = 0.0;
      auto & color = marker.colors.emplace_back();
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 1.0;
    }
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    ss_vis_pub_->publish(ss_vis_msg);
  }

  // publish the telemetry message
  telemetry_msg.header.stamp = now;
  mpc_telemetry_pub_->publish(telemetry_msg);
}

rcl_interfaces::msg::SetParametersResult RacingMPCNode::on_set_parameters(
  std::vector<rclcpp::Parameter> const & parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = false;
  // only accept velocity scale changes
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "racing_mpc_node.velocity_profile_scale") {
      const auto speed_scale = parameter.as_double();
      if (speed_scale < 0.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid velocity scale %f, must be non-negative", speed_scale);
        return result;
      }
      
      set_speed_scale(speed_scale);
      RCLCPP_INFO(
        this->get_logger(),
        "Set velocity scale to %f", speed_scale_);
      result.successful = true;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot set parameter %s", parameter.get_name().c_str());
    }
  }
  return result;
}

void RacingMPCNode::change_trajectory(const int & traj_idx) {
  if (traj_idx == traj_idx_) {
    return;
  }

  std::unique_lock<std::shared_mutex> traj_lock(traj_mutex_);
  auto & old_traj = *track_;
  auto new_traj = tracks_->get_trajectory(traj_idx);

  if (new_traj) {
    auto convert_frame = [&](const FrenetPose2D & old_frenet_pose) {
      Pose2D global_pose;
      old_traj.frenet_to_global(old_frenet_pose, global_pose);
      FrenetPose2D new_frenet_pose;
      new_traj->global_to_frenet(global_pose, new_frenet_pose);
      return new_frenet_pose;
    };
    if (vehicle_state_msg_) {
      std::unique_lock<std::shared_mutex> state_msg_lock(state_msg_mutex_);
      auto & x = vehicle_state_msg_->x;
      auto & e = vehicle_state_msg_->e;
      // convert current pose to global then to new coordinate system
      const Pose2D old_global_pose {{x.x, x.y}, e.psi};
      FrenetPose2D new_frenet_pose;
      new_traj->global_to_frenet(old_global_pose, new_frenet_pose);
      vehicle_state_msg_->p.s = new_frenet_pose.position.s;
      vehicle_state_msg_->p.x_tran = new_frenet_pose.position.t;
      vehicle_state_msg_->p.e_psi = new_frenet_pose.yaw;
      state_msg_lock.unlock();

      // convert previous solution to new coordinate system
      if (mpc_->solved()) {
        for (int i = 0; i< config_->N; i++) {
          const auto xi = last_x_(casadi::Slice(), i).get_elements();
          const FrenetPose2D old_frenet_pose {{xi[XIndex::PX], xi[XIndex::PY]}, xi[XIndex::YAW]};
          const FrenetPose2D new_frenet_pose = convert_frame(old_frenet_pose);
          last_x_(XIndex::PX, i) = new_frenet_pose.position.s;
          last_x_(XIndex::PY, i) = new_frenet_pose.position.t;
          last_x_(XIndex::YAW, i) = new_frenet_pose.yaw;
        }
      }
    }
    track_ = new_traj;
    f2g_ = track_->frenet_to_global_function().map(mpc_->get_config().N);
    traj_idx_ = traj_idx;
    vis_ = std::make_unique<ROSTrajectoryVisualizer>(*track_);
    vis_->attach_ros_publishers(this, 1.0, true, true);
    sol_in_["total_length"] = track_->total_length();

    // build discrete dynamics
    const auto x_sym = casadi::MX::sym("x", model_->nx());
    const auto u_sym = casadi::MX::sym("u", model_->nu());
    const auto k = track_->curvature_interpolation_function()(x_sym(XIndex::PX))[0];
    const auto xip1 = model_->discrete_dynamics()(
      casadi::MXDict{{"x", x_sym}, {"u", u_sym}, {"k", k}, {"dt", dt_}}
    ).at("xip1");
    discrete_dynamics_ = casadi::Function("discrete_dynamics", {x_sym, u_sym}, {xip1});

    RCLCPP_INFO(
      this->get_logger(),
      "Changed trajectory to %d.", traj_idx_);
  }
}

void RacingMPCNode::set_speed_limit(const double & speed_limit)
{
  std::unique_lock<std::shared_mutex> speed_limit_lock(speed_limit_mutex_);
  this->speed_limit_ = speed_limit;
  speed_limit_lock.unlock();
  RCLCPP_INFO(
    this->get_logger(),
    "Set speed limit to %f m/s.", speed_limit_);
}

void RacingMPCNode::set_speed_scale(const double & speed_scale)
{
  double scale = 0.2;
  if (speed_scale > 1.0 || speed_scale <= 0.0)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid velocity scale %f, must be between (0.0-1.0]. Resetting to 20\%.", speed_scale);
  } else {
    scale = speed_scale;
    RCLCPP_INFO(
      this->get_logger(),
      "Set velocity scale to %f", speed_scale);
  }
  std::unique_lock<std::shared_mutex> speed_scale_lock(speed_scale_mutex_);
  this->speed_scale_ = scale;
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
