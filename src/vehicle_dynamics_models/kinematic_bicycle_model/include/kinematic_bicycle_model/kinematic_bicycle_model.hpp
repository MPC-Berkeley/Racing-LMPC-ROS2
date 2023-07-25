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

#ifndef KINEMATIC_BICYCLE_MODEL__KINEMATIC_BICYCLE_MODEL_HPP_
#define KINEMATIC_BICYCLE_MODEL__KINEMATIC_BICYCLE_MODEL_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "base_vehicle_model/base_vehicle_model.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace kinematic_bicycle_model
{
struct KinematicBicycleModelConfig
{
  typedef std::shared_ptr<KinematicBicycleModelConfig> SharedPtr;

  double Fd_max;
  double Fb_max;
  double Td;
  double Tb;
  double v_max;
  double P_max;
  double mu;
};

enum XIndex : casadi_int
{
  PX = 0,  // global or frenet x position
  PY = 1,  // global or frenet y position
  YAW = 2,  // global or frenet yaw
  V = 3,  // body velocity
};

enum UIndex : casadi_int
{
  FD = 0,
  FB = 1,
  STEER = 2
};

class KinematicBicycleModel final : public base_vehicle_model::BaseVehicleModel
{
public:
  typedef std::shared_ptr<KinematicBicycleModel> SharedPtr;
  typedef std::unique_ptr<KinematicBicycleModel> UniquePtr;

  KinematicBicycleModel(
    base_vehicle_model::BaseVehicleModelConfig::SharedPtr base_config,
    KinematicBicycleModelConfig::SharedPtr config);

  const KinematicBicycleModelConfig & get_config() const;
  casadi::Function & dynamics() {return dynamics_;}

  size_t nx() const override;
  size_t nu() const override;

  void forward_dynamics(const casadi::DMDict & in, casadi::DMDict & out) override;
  void dynamics_jacobian(const casadi::DMDict & in, casadi::DMDict & out) override;
  void add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in) override;
  void calc_lon_control(
    const casadi::DMDict & in, double & throttle,
    double & brake_kpa) const override;
  void calc_lat_control(const casadi::DMDict & in, double & steering_rad) const override;

private:
  void compile_dynamics();

  KinematicBicycleModelConfig::SharedPtr config_ {};
  casadi::Function dynamics_;
  casadi::Function dynamics_jac_;
};
}  // namespace kinematic_bicycle_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // KINEMATIC_BICYCLE_MODEL__KINEMATIC_BICYCLE_MODEL_HPP_
