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

#ifndef BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_CONFIG_HPP_
#define BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_CONFIG_HPP_

#include <memory>
#include <vector>

#include <lmpc_utils/lookup.hpp>

namespace lmpc
{
namespace vehicle_model
{
namespace base_vehicle_model
{
struct TyreConfig
{
  typedef std::shared_ptr<TyreConfig> SharedPtr;
  double radius;  // m
  double width;  // m
  double mass;  // kg
  double moi;  // rotational moment of inertia in kg * m^2

  double pacejka_b;  // Pacejka magic number
  double pacejka_c;  // Pacejka magic number
  double pacejka_e;  // Pacejka magic number
  double pacejka_fz0;  // Nominal normal force (N)
  double pacejka_eps;  // Extended Pacejka degradation coefficients
};

struct BrakeConfig
{
  typedef std::shared_ptr<BrakeConfig> SharedPtr;
  double max_brake;  // kpa
  double brake_pad_out_r;  // brake pad outter radius (m)
  double brake_pad_in_r;  // brake pad inner radius (m)
  double brake_pad_friction_coeff;  // brake pad kinetic friction coefficient
  double piston_area;  // Sum of all brake pistons' area (m^2)
  double bias;  // ratio of total brake force
};

struct SteerConfig
{
  typedef std::shared_ptr<SteerConfig> SharedPtr;
  double max_steer_rate;  // steering rate at wheel (rad/s)
  double max_steer;  // positive left (rad)
  double turn_left_bias;  // positive left (rad)
};

struct ChassisConfig
{
  typedef std::shared_ptr<ChassisConfig> SharedPtr;
  double total_mass;  // kg
  double sprung_mass;  // kg
  double unsprung_mass;  // kg
  double cg_ratio;  // ratio of car weight on front axle
  double cg_height;  // cg height from ground (m)
  double wheel_base;  // m
  double tw_f;  // front track width (m)
  double tw_r;  // rear track width (m)
  double moi;  // polar moment of inertia (kg * m^2)
  double b;  // vehicle width (m)
  double fr;  // rolling resistance coefficient
};

struct AeroConfig
{
  typedef std::shared_ptr<AeroConfig> SharedPtr;

  // drag
  double air_density;  // kg/m^3
  double drag_coeff;
  double frontal_area;  // m^2

  // downforce
  double cl_f;  // downforce coefficient at front axle
  double cl_r;  // downforce coefficient at rear axle
};

// TODO(haoru): add suspension config when upgrading to 6 DOF model

struct PowerTrainConfig
{
  typedef std::shared_ptr<PowerTrainConfig> SharedPtr;

  // lookup of torque (N*m) against RPM (rev/min) and throttle (0-100)
  lmpc::utils::Lookup3D torque_v_rpm_throttle;

  // Gear ratio of each gear (beginning at 1st)
  std::vector<double> gear_ratio;

  // Differential final drive ratio
  double final_drive_ratio;

  // Drive force distribution at front axle as a fraction of total drive force
  double kd;

  // TODO(haoru): add mechanical loss
};

enum IntegratorType : uint8_t
{
  RK4,
  EULER
};

/**
 * @brief Additional configurations that doesn't really concern the vehicle parameters
 *
 */
struct ModelingConfig
{
  typedef std::shared_ptr<ModelingConfig> SharedPtr;

  // Express the dynamics in frenet frame?
  bool use_frenet;

  // Integrator type
  IntegratorType integrator_type;

  // Sample throttle point for torque lookup (0-100)
  double sample_throttle;
};

struct BaseVehicleModelConfig
{
  typedef std::shared_ptr<BaseVehicleModelConfig> SharedPtr;

  base_vehicle_model::TyreConfig::SharedPtr front_tyre_config;
  base_vehicle_model::TyreConfig::SharedPtr rear_tyre_config;
  base_vehicle_model::BrakeConfig::SharedPtr front_brake_config;
  base_vehicle_model::BrakeConfig::SharedPtr rear_brake_config;
  base_vehicle_model::SteerConfig::SharedPtr steer_config;
  base_vehicle_model::ChassisConfig::SharedPtr chassis_config;
  base_vehicle_model::AeroConfig::SharedPtr aero_config;
  base_vehicle_model::PowerTrainConfig::SharedPtr powertrain_config;
  base_vehicle_model::ModelingConfig::SharedPtr modeling_config;
};
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_CONFIG_HPP_
