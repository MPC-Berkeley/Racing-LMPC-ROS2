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

#include <math.h>
#include <exception>
#include <vector>
#include <iostream>
#include <chrono>

#include <lmpc_utils/utils.hpp>
#include <lmpc_transform_helper/lmpc_transform_helper.hpp>

#include "vanilla_controller/vanilla_controller.hpp"

#define GRAVITY 9.81

namespace lmpc
{
namespace mpc
{
namespace vanilla_controller
{
VanillaController::VanillaController(
  VanillaControllerConfig::SharedPtr controller_config,
  BaseVehicleModel::SharedPtr model,
  RacingTrajectory::SharedPtr track)
: config_(controller_config), model_(model), track_(track), pid_controller_("lon_pid",
    config_->lon_pid_coeffs)
{
}

const VanillaControllerConfig & VanillaController::get_config() const
{
  return *config_.get();
}

void VanillaController::solve(const casadi::DMDict & in, casadi::DMDict & out, casadi::Dict & stats)
{
  (void) stats;
  const auto x_ic = in.at("x_ic");
  const auto u_ic = in.at("u_ic");
  const auto vel_ref = in.at("vel_ref");
  const auto x_ic_vec = x_ic.get_elements();
  auto u_out = casadi::DM::zeros(model_->nu(), 1);
  const auto & chassis = model_->get_base_config().chassis_config;
  const auto & aero = model_->get_base_config().aero_config;
  const auto & steer = model_->get_base_config().steer_config;

  // find current global pose
  FrenetPose2D current_frenet_pose{{x_ic_vec[XIndex::PX], x_ic_vec[XIndex::PY]},
    x_ic_vec[XIndex::YAW]};
  Pose2D current_global_pose;
  track_->frenet_to_global(current_frenet_pose, current_global_pose);

  // find pure pursuit target
  const auto s = x_ic(XIndex::PX);
  const auto v = static_cast<double>(hypot(x_ic(XIndex::VX), x_ic(XIndex::VY)));
  const auto lookahead_dist = std::clamp(
    v * config_->lookahead_speed_ratio,
    config_->min_lookahead_distance, config_->max_lookahead_distance);
  const auto s_lookahead = utils::align_abscissa<casadi::DM>(
    s + lookahead_dist,
    track_->total_length() / 2.0, track_->total_length());
  FrenetPose2D lookahead_frenet_pose{{static_cast<double>(s_lookahead), 0.0}, 0.0};
  Pose2D lookahead_global_pose;
  track_->frenet_to_global(lookahead_frenet_pose, lookahead_global_pose);

  // compute pure pursuit
  const auto lookahead_direction = atan2(
    lookahead_global_pose.position.y - current_global_pose.position.y,
    lookahead_global_pose.position.x - current_global_pose.position.x);
  const auto alpha = utils::TransformHelper::calc_yaw_difference(
    current_global_pose.yaw,
    lookahead_direction);
  const auto delta = atan(2.0 * chassis->wheel_base * sin(alpha) / lookahead_dist);
  const auto delta_clamped = std::clamp(delta, -steer->max_steer, steer->max_steer);
  u_out(UIndex::STEER) = delta_clamped;

  // compute longitudinal control
  const auto vel_error = static_cast<double>(vel_ref(0)) - v;
  const auto acc = pid_controller_.update(vel_error, config_->dt);
  const auto aero_resistance = 0.5 * aero->air_density * aero->frontal_area * aero->drag_coeff * v *
    v;
  const auto down_force = aero_resistance * (aero->cl_f + aero->cl_r);
  const auto rolling_resistance = chassis->fr * (chassis->total_mass * GRAVITY + down_force);
  const auto ctrl_force = chassis->total_mass * acc + rolling_resistance + aero_resistance;
  if (ctrl_force > 0.0) {
    u_out(UIndex::FD) = ctrl_force;
    u_out(UIndex::FB) = 0.0;
  } else {
    u_out(UIndex::FD) = 0.0;
    u_out(UIndex::FB) = ctrl_force;
  }

  // output
  out["u_out"] = u_out;
}

BaseVehicleModel & VanillaController::get_model()
{
  return *model_;
}
}  // namespace vanilla_controller
}  // namespace mpc
}  // namespace lmpc
