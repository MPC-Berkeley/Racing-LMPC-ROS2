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

#include "base_vehicle_model/base_vehicle_model.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace base_vehicle_model
{
BaseVehicleModel::BaseVehicleModel(BaseVehicleModelConfig::SharedPtr config)
: base_config_(config)
{
}

void BaseVehicleModel::set_base_config(BaseVehicleModelConfig::SharedPtr config)
{
  base_config_ = config;
}

const BaseVehicleModelConfig & BaseVehicleModel::get_base_config() const
{
  return *base_config_.get();
}

BaseVehicleModelState & BaseVehicleModel::get_state()
{
  return base_state_;
}

const BaseVehicleModelState & BaseVehicleModel::get_const_state() const
{
  return base_state_;
}

size_t BaseVehicleModel::nx() const
{
  return 1;
}

size_t BaseVehicleModel::nu() const
{
  return 1;
}

void BaseVehicleModel::forward_dynamics(const casadi::DMDict & in, casadi::DMDict & out)
{
  (void) in;
  (void) out;
}

void BaseVehicleModel::dynamics_jacobian(const casadi::DMDict & in, casadi::DMDict & out)
{
  (void) in;
  (void) out;
}

void BaseVehicleModel::add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in)
{
  (void) opti;
  (void) in;
}

void BaseVehicleModel::calc_lon_control(
  const casadi::DMDict & in, double & throttle,
  double & brake_kpa) const
{
  (void) in;
  (void) throttle;
  (void) brake_kpa;
}

void BaseVehicleModel::calc_lat_control(const casadi::DMDict & in, double & steering_rad) const
{
  (void) in;
  (void) steering_rad;
}

double BaseVehicleModel::calc_throttle(const double & fd) const
{
  if (fd < 0.0) {
    return 0.0;
  }
  const auto & pt_config = *base_config_->powertrain_config.get();

  if (base_state_.gear > pt_config.gear_ratio.size()) {
    printf("Gear number of %d is not possible.", base_state_.gear);
    return 0.0;
  }

  const auto target_front_wheel_torque = fd * base_config_->front_tyre_config->radius *
    pt_config.kd;
  const auto target_rear_wheel_torque = fd * base_config_->rear_tyre_config->radius *
    (1 - pt_config.kd);
  const auto target_engine_torque = (target_front_wheel_torque + target_rear_wheel_torque) /
    (pt_config.gear_ratio[base_state_.gear - 1] * pt_config.final_drive_ratio);
  const auto min_engine_torque = bilinear_interpolate(
    pt_config.torque_v_rpm_throttle, base_state_.engine_rpm, 0.0, false);
  const auto max_engine_torque = bilinear_interpolate(
    pt_config.torque_v_rpm_throttle, base_state_.engine_rpm, 100.0, false);
  return utils::fast_linear_interpolate(
    min_engine_torque, max_engine_torque, 0.0, 100.0,
    target_engine_torque, false);
}

double BaseVehicleModel::calc_brake(const double & fb) const
{
  if (fb > 0.0) {
    return 0.0;
  }
  const auto & fb_config = *base_config_->front_brake_config.get();
  const auto & rb_config = *base_config_->rear_brake_config.get();
  const auto front_control_torque = fb_config.bias * fb *
    base_config_->front_tyre_config->radius * fb_config.bias;
  const auto rear_control_torque = rb_config.bias * fb * base_config_->rear_tyre_config->radius *
    rb_config.bias;
  const auto front_brake_lever =
    (fb_config.brake_pad_in_r + fb_config.brake_pad_out_r) /
    2.0;
  const auto rear_brake_lever =
    (rb_config.brake_pad_in_r + rb_config.brake_pad_out_r) /
    2.0;

  // T = lever * friction_mu * pressure * pi * piston_radius^2 * num_pad
  const auto front_brake_kpa = -0.001 * front_control_torque /
    (front_brake_lever * fb_config.brake_pad_friction_coeff *
    fb_config.piston_area);
  const auto rear_brake_kpa = -0.001 * rear_control_torque /
    (rear_brake_lever * rb_config.brake_pad_friction_coeff *
    rb_config.piston_area);

  return std::clamp(front_brake_kpa, 0.0, fb_config.max_brake) +
         std::clamp(rear_brake_kpa, 0.0, rb_config.max_brake);
}
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
