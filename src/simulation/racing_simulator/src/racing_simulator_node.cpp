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

#include <base_vehicle_model/ros_param_loader.hpp>
#include <single_track_planar_model/ros_param_loader.hpp>

#include "racing_simulator/racing_simulator_node.hpp"
#include "racing_simulator/ros_param_loader.hpp"

namespace lmpc
{
namespace simulation
{
namespace racing_simulator
{
RacingSimulatorNode::RacingSimulatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("racing_simulator_node", options),
  config_(lmpc::simulation::racing_simulator::load_parameters(this)),
  track_(std::make_shared<RacingTrajectory>(config_->race_track_file_path)),
  sim_step_(0),
  lap_count_(0),
  tf_helper_(*this)
{
  // initialize simulator
  auto base_model_config =
    lmpc::vehicle_model::base_vehicle_model::load_parameters(this);
  auto single_track_model_config =
    lmpc::vehicle_model::single_track_planar_model::load_parameters(this);
  base_model_config->modeling_config->use_frenet = config_->use_frenet;
  model_ = std::make_shared<SingleTrackPlanarModel>(base_model_config, single_track_model_config);
  simulator_ = std::make_shared<RacingSimulator>(config_->dt, config_->x0, track_, model_);

  // build the cg to base_link transform
  const auto & chassis_config = *(model_->get_base_config().chassis_config);
  const auto & lr = chassis_config.wheel_base * chassis_config.cg_ratio;
  const auto & lf = chassis_config.wheel_base - lr;
  cg_to_baselink_.setOrigin(tf2::Vector3(-1.0 * lr, 0.0, 0.0));
  cg_to_baselink_.setRotation(utils::TransformHelper::quaternion_from_heading(0.0));

  // initialize vehicle state message
  vehicle_state_msg_ = std::make_shared<mpclab_msgs::msg::VehicleStateMsg>();
  const auto x0_vec = config_->x0.get_elements();
  FrenetPose2D frenet_pose;
  Pose2D global_pose;
  if (config_->use_frenet) {
    frenet_pose.position.s = x0_vec[XIndex::PX];
    frenet_pose.position.t = x0_vec[XIndex::PY];
    frenet_pose.yaw = x0_vec[XIndex::YAW];
    track_->frenet_to_global(frenet_pose, global_pose);
  } else {
    global_pose.position.x = x0_vec[XIndex::PX];
    global_pose.position.y = x0_vec[XIndex::PY];
    global_pose.yaw = x0_vec[XIndex::YAW];
    track_->global_to_frenet(global_pose, frenet_pose);
  }
  update_vehicle_state_msg(x0_vec, frenet_pose, global_pose);

  // initialize the map to base_link message
  if (config_->publish_tf) {
    map_to_baselink_msg_ = std::make_shared<TransformStamped>();
    map_to_baselink_msg_->header.frame_id = "map";
    map_to_baselink_msg_->child_frame_id = "base_link";

    tf2::Transform map_to_cg;
    map_to_cg.setOrigin(tf2::Vector3(global_pose.position.x, global_pose.position.y, 0.0));
    map_to_cg.setRotation(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
    map_to_baselink_msg_->transform = tf2::toMsg(map_to_cg);
  }

  // build the visualization polygons
  if (config_->visualize_vehicle) {
    vehicle_polygon_msg_ = std::make_shared<PolygonStamped>();
    vehicle_polygon_msg_->header.frame_id = "base_link";
    const auto half_width = chassis_config.b / 2.0;
    const casadi::DM pts = casadi::DM::reshape(
      casadi::DM(
          {
            lf, half_width,
            lf, -half_width,
            -lr, -half_width,
            -lr, half_width
          }), 2, 4);
    vehicle_polygon_msg_->polygon = build_polygon(pts);
  }
  const auto abscissa =
    casadi::DM::linspace(0.0, track_->total_length(), 1001)(casadi::Slice(0, -1));
  const auto N = abscissa.size1();
  if (config_->visualize_abscissa) {
    abscissa_polygon_msg_ = std::make_shared<PolygonStamped>();
    abscissa_polygon_msg_->header.frame_id = "map";
    const auto pts = casadi::DM::horzcat(
          {
            track_->x_interpolation_function()(abscissa)[0],
            track_->y_interpolation_function()(abscissa)[0]
          }).T();
    abscissa_polygon_msg_->polygon = build_polygon(pts);
  }
  if (config_->visualize_boundary) {
    left_boundary_polygon_msg_ = std::make_shared<PolygonStamped>();
    left_boundary_polygon_msg_->header.frame_id = "map";
    right_boundary_polygon_msg_ = std::make_shared<PolygonStamped>();
    right_boundary_polygon_msg_->header.frame_id = "map";
    const auto & left_boundary_frenet = casadi::DM::horzcat(
          {
            abscissa,
            track_->left_boundary_interpolation_function()(abscissa)[0],
            casadi::DM::zeros(N)
          }).T();
    const auto & right_boundary_frenet = casadi::DM::horzcat(
          {
            abscissa,
            track_->right_boundary_interpolation_function()(abscissa)[0],
            casadi::DM::zeros(N)
          }).T();
    const auto left_boundary_global =
      track_->frenet_to_global_function().map(N)(left_boundary_frenet)[0];
    const auto right_boundary_global =
      track_->frenet_to_global_function().map(N)(right_boundary_frenet)[0];
    left_boundary_polygon_msg_->polygon = build_polygon(left_boundary_global);
    right_boundary_polygon_msg_->polygon = build_polygon(right_boundary_global);
  }

  // initialize state publishers
  vehicle_state_pub_ = this->create_publisher<mpclab_msgs::msg::VehicleStateMsg>(
    "vehicle_state", 1);

  // initialize visualization publishers
  if (config_->visualize_vehicle) {
    vehicle_polygon_pub_ = this->create_publisher<PolygonStamped>(
      "vehicle_polygon", 1);
  }
  if (config_->visualize_boundary) {
    left_boundary_polygon_pub_ = this->create_publisher<PolygonStamped>(
      "left_boundary_polygon", 1);
    right_boundary_polygon_pub_ = this->create_publisher<PolygonStamped>(
      "right_boundary_polygon", 1);
  }
  if (config_->visualize_abscissa) {
    abscissa_polygon_pub_ = this->create_publisher<PolygonStamped>(
      "abscissa_polygon", 1);
  }
  vehicle_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "vehicle_odom", 1);

  // initialize subscribers
  vehicle_actuation_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleActuationMsg>(
    "vehicle_actuation", 1,
    std::bind(&RacingSimulatorNode::on_actuation, this, std::placeholders::_1));
  reset_state_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleStateMsg>(
    "reset_state", 1,
    std::bind(&RacingSimulatorNode::on_reset_state, this, std::placeholders::_1));

  // initialize timer
  static_vis_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&RacingSimulatorNode::on_static_vis_timer, this));

  if (config_->repeat_state_dt > 0) {
    state_repub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(config_->repeat_state_dt * 1000.0)),
      std::bind(&RacingSimulatorNode::on_state_repub_timer, this));
  }

  if (config_->step_mode == RacingSimulatorStepMode::CONTINUOUS) {
    sim_step_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(config_->dt * 1000.0)),
      std::bind(&RacingSimulatorNode::on_state_update, this));
  }
}

void RacingSimulatorNode::on_actuation(const mpclab_msgs::msg::VehicleActuationMsg::SharedPtr msg)
{
  vehicle_actuation_msg_ = msg;
  // const auto start = std::chrono::high_resolution_clock::now();
  if (config_->step_mode == RacingSimulatorStepMode::STEP) {
    on_state_update();
  }
  // const auto stop = std::chrono::high_resolution_clock::now();
  // const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << "Simulation Step Time: " << duration.count() << "us" << std::endl;
}

void RacingSimulatorNode::on_reset_state(const mpclab_msgs::msg::VehicleStateMsg::SharedPtr msg)
{
  // reset the simulator
  casadi::DM x;
  if (config_->use_frenet) {
    x = casadi::DM{
      msg->p.s,
      msg->p.x_tran,
      msg->p.e_psi,
      msg->v.v_long,
      msg->v.v_tran,
      msg->w.w_psi
    };
  } else {
    x = casadi::DM{
      msg->x.x,
      msg->x.y,
      msg->e.psi,
      msg->v.v_long,
      msg->v.v_tran,
      msg->w.w_psi
    };
  }
  simulator_->set_state(x);
  // sim_step_ = 0;
  // lap_count_ = msg->lap_num;
}

void RacingSimulatorNode::on_static_vis_timer()
{
  const auto now = this->now();
  if (config_->visualize_abscissa) {
    abscissa_polygon_msg_->header.stamp = now;
    abscissa_polygon_pub_->publish(*abscissa_polygon_msg_);
  }
  if (config_->visualize_boundary) {
    left_boundary_polygon_msg_->header.stamp = now;
    left_boundary_polygon_pub_->publish(*left_boundary_polygon_msg_);
    right_boundary_polygon_msg_->header.stamp = now;
    right_boundary_polygon_pub_->publish(*right_boundary_polygon_msg_);
  }
}

void RacingSimulatorNode::on_state_repub_timer()
{
  RCLCPP_INFO(
    this->get_logger(), "No actuation message received for %f seconds, republishing the state.",
    config_->repeat_state_dt);
  const auto now = this->now();
  vehicle_state_msg_->header.stamp = now;
  vehicle_state_pub_->publish(*vehicle_state_msg_);

  if (config_->publish_tf) {
    map_to_baselink_msg_->header.stamp = now;
    tf_helper_.send_transform(*map_to_baselink_msg_);
  }
  if (config_->visualize_vehicle) {
    vehicle_polygon_msg_->header.stamp = now;
    vehicle_polygon_pub_->publish(*vehicle_polygon_msg_);
  }
}

Polygon RacingSimulatorNode::build_polygon(const casadi::DM & pts)
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

void RacingSimulatorNode::update_vehicle_state_msg(
  const std::vector<double> & x,
  const FrenetPose2D & frenet_pose,
  const Pose2D & global_pose)
{
  // calculate the frenet frame velocity
  const auto k = static_cast<double>(
    track_->curvature_interpolation_function()(casadi::DM(frenet_pose.position.s))[0]);
  const auto vb = BodyVelocity2D{x[XIndex::VX], x[XIndex::VY], x[XIndex::VYAW]};
  auto vs = transform_velocity(vb, frenet_pose.yaw);
  vs.x /= (1.0 - k * frenet_pose.position.t);

  // build the updated state message
  vehicle_state_msg_->t = sim_step_ * config_->dt;
  vehicle_state_msg_->x.x = global_pose.position.x;
  vehicle_state_msg_->x.y = global_pose.position.y;
  vehicle_state_msg_->e.psi = global_pose.yaw;
  // TODO(haoru): populate body accel
  vehicle_state_msg_->v.v_long = x[XIndex::VX];
  vehicle_state_msg_->v.v_tran = x[XIndex::VY];
  vehicle_state_msg_->p.s = frenet_pose.position.s;
  vehicle_state_msg_->p.x_tran = frenet_pose.position.t;
  vehicle_state_msg_->p.e_psi = frenet_pose.yaw;
  vehicle_state_msg_->pt.ds = vs.x;
  vehicle_state_msg_->pt.dx_tran = vs.y;
  vehicle_state_msg_->w.w_psi = x[XIndex::VYAW];
  vehicle_state_msg_->pt.de_psi = x[XIndex::VYAW] - vs.x * k;

  if (vehicle_actuation_msg_) {
    vehicle_state_msg_->u = *vehicle_actuation_msg_;
  } else {
    vehicle_state_msg_->u.u_a = 0.0;
    vehicle_state_msg_->u.u_steer = 0.0;
  }
  vehicle_state_msg_->lap_num = lap_count_ + frenet_pose.position.s / track_->total_length();
}

void RacingSimulatorNode::on_state_update()
{
  if (!vehicle_actuation_msg_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Waiting for vehicle actuation message.");
    return;
  }
  const auto last_x = simulator_->x().get_elements();
  const auto u = casadi::DM(
        {
          vehicle_actuation_msg_->u_a > 0.0 ? vehicle_actuation_msg_->u_a : 0.0,
          vehicle_actuation_msg_->u_a < 0.0 ? vehicle_actuation_msg_->u_a : 0.0,
          vehicle_actuation_msg_->u_steer
        });
  simulator_->step(u);

  // get the updated state
  const auto x = simulator_->x().get_elements();
  // std::cout << "x: " << last_x << std::endl;
  // std::cout << "u: " << u << std::endl;
  // std::cout << "xip1: " << x << std::endl << std::endl;

  // increment simulation step
  sim_step_++;

  // calculate the global pose
  FrenetPose2D frenet_pose;
  Pose2D global_pose;
  if (config_->use_frenet) {
    frenet_pose.position.s = x[XIndex::PX];
    frenet_pose.position.t = x[XIndex::PY];
    frenet_pose.yaw = x[XIndex::YAW];
    track_->frenet_to_global(frenet_pose, global_pose);
  } else {
    global_pose.position.x = x[XIndex::PX];
    global_pose.position.y = x[XIndex::PY];
    global_pose.yaw = x[XIndex::YAW];
    track_->global_to_frenet(global_pose, frenet_pose);
  }

  // increment lap count if necessary
  if (vehicle_state_msg_->p.s - frenet_pose.position.s > 0.5 * track_->total_length()) {
    lap_count_++;
  }

  // update the vehicle state message
  const auto now = this->now();
  vehicle_state_msg_->header.stamp = now;
  update_vehicle_state_msg(x, frenet_pose, global_pose);

  // publish tf
  if (config_->publish_tf) {
    // build the map to cg transform
    tf2::Transform map_to_cg;
    map_to_cg.setOrigin(tf2::Vector3(global_pose.position.x, global_pose.position.y, 0.0));
    map_to_cg.setRotation(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
    // find the map to baselink transform
    map_to_baselink_msg_->transform = tf2::toMsg(map_to_cg * cg_to_baselink_);
    map_to_baselink_msg_->header.stamp = now;
    // publish the transforms
    tf_helper_.send_transform(*map_to_baselink_msg_);
  }

  // publish odom
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = global_pose.position.x;
  odom.pose.pose.position.y = global_pose.position.y;
  odom.pose.pose.orientation =
    tf2::toMsg(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
  odom.twist.twist.linear.x = vehicle_state_msg_->v.v_long;
  odom.twist.twist.linear.y = vehicle_state_msg_->v.v_tran;
  odom.twist.twist.angular.z = vehicle_state_msg_->w.w_psi;
  vehicle_odom_pub_->publish(odom);

  // skip the repub timer
  if (state_repub_timer_) {
    state_repub_timer_->reset();
  }

  // publish the updated state
  vehicle_state_pub_->publish(*vehicle_state_msg_);

  // publish the vehicle visualization
  if (config_->visualize_vehicle) {
    vehicle_polygon_msg_->header.stamp = now;
    vehicle_polygon_pub_->publish(*vehicle_polygon_msg_);
  }
}
}  // namespace racing_simulator
}  // namespace simulation
}  // namespace lmpc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lmpc::simulation::racing_simulator::RacingSimulatorNode)
