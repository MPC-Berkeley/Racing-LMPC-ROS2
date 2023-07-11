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

#include "racing_simulator/racing_simulator_node.hpp"
#include "racing_simulator/ros_param_loader.hpp"
#include "single_track_planar_model/ros_param_loader.hpp"

namespace lmpc
{
namespace simulation
{
namespace racing_simulator
{
RacingSimulatorNode::RacingSimulatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("basestation_race_control", options),
config_(lmpc::simulation::racing_simulator::load_parameters(this)),
track_(std::make_shared<RacingTrajectory>(config_->race_track_file_path)),
model_(std::make_shared<SingleTrackPlanarModel>(lmpc::vehicle_model::single_track_planar_model::load_parameters(this))),
simulator_(std::make_shared<RacingSimulator>(config_->dt, config_->x0, track_, model_)),
tf_helper_(*this),
sim_step_(0),
lap_count_(0)
{
    const auto & chassis_config = *(model_->get_base_config().chassis_config);
    const auto x = -1.0 * chassis_config.wheel_base * chassis_config.cg_ratio;
    cg_to_baselink_.setOrigin(tf2::Vector3(x, 0.0, 0.0));
    // initialize state publishers
    vehicle_state_pub_ = this->create_publisher<mpclab_msgs::msg::VehicleStateMsg>(
        "vehicle_state", 1);
    
    // initialize visualization publishers
    if (config_->visualize_vehicle) {
        vehicle_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "vehicle_polygon", 1);
    }
    if (config_->visualize_boundary) {
        left_boundary_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "left_boundary_polygon", 1);
        right_boundary_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "right_boundary_polygon", 1);
    }
    if (config_->visualize_abscissa) {
        abscissa_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
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
}

void RacingSimulatorNode::on_actuation(const mpclab_msgs::msg::VehicleActuationMsg::SharedPtr msg)
{
    const auto last_x = simulator_->x().get_elements();
    const auto u = casadi::DM({
        msg->u_a > 0.0 ? msg->u_a : 0.0,
        msg->u_a < 0.0 ? msg->u_a : 0.0,
        msg->u_steer
    });
    simulator_->step(u);

    // get the updated state
    const auto x = simulator_->x().get_elements();

    // increment lap count if necessary
    if (last_x[XIndex::PX] - x[XIndex::PX] > 0.5 * track_->total_length()) {
        lap_count_++;
    }

    // increment simulation step
    sim_step_++;

    // calculate the global pose
    Pose2D global_pose;
    track_->frenet_to_global({x[XIndex::PX], x[XIndex::PY], x[XIndex::YAW]}, global_pose);
    
    // build and publish the updated state message
    const auto now = this->now();
    mpclab_msgs::msg::VehicleStateMsg state_msg;
    state_msg.header.stamp = now;
    state_msg.t = sim_step_ * config_->dt;
    state_msg.x.x = global_pose.position.x;
    state_msg.x.y = global_pose.position.y;
    state_msg.e.phi = global_pose.yaw;
    // TODO(haoru): populate body velocity and accel
    state_msg.p.s = x[XIndex::PX];
    state_msg.p.x_tran = x[XIndex::PY];
    state_msg.p.e_psi = x[XIndex::YAW];
    state_msg.pt.ds = x[XIndex::V] * cos(x[XIndex::SLIP]);
    state_msg.pt.dx_tran = x[XIndex::V] * sin(x[XIndex::SLIP]);
    state_msg.pt.de_psi = x[XIndex::V_YAW];
    state_msg.u = *msg;
    state_msg.lap_num = lap_count_ + x[XIndex::PX] / track_->total_length();
    vehicle_state_pub_->publish(state_msg);

    // publish tf
    if (config_->publish_tf) {
        // build the map to cg transform
        geometry_msgs::msg::TransformStamped map_to_cg;
        map_to_cg.header.stamp = now;
        map_to_cg.header.frame_id = "map";
        map_to_cg.child_frame_id = "cg";
        map_to_cg.transform.translation.x = global_pose.position.x;
        map_to_cg.transform.translation.y = global_pose.position.y;
        map_to_cg.transform.rotation = tf2::toMsg(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
        // find the map to baselink transform
        tf2::Transform map_to_cg_tf2;
        tf2::fromMsg(map_to_cg, map_to_cg_tf2);
        const auto map_to_baselink = tf2::toMsg(map_to_cg_tf2 * cg_to_baselink_);
        // publish the transforms
        tf_helper_.send_transform(map_to_cg);
    }

    // publish visualization 
}
}  // namespace racing_simulator
}  // namespace simulation
}  // namespace lmpc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lmpc::simulation::racing_simulator::RacingSimulatorNode)
