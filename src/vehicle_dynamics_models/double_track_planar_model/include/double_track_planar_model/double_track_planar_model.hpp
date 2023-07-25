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

#ifndef DOUBLE_TRACK_PLANAR_MODEL__DOUBLE_TRACK_PLANAR_MODEL_HPP_
#define DOUBLE_TRACK_PLANAR_MODEL__DOUBLE_TRACK_PLANAR_MODEL_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "base_vehicle_model/base_vehicle_model.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace double_track_planar_model
{
struct DoubleTrackPlanarModelConfig
{
  typedef std::shared_ptr<DoubleTrackPlanarModelConfig> SharedPtr;

  double Fd_max;
  double Fb_max;
  double Td;
  double Tb;
  double v_max;
  double P_max;
  double kroll_f;
  double mu;
};

enum XIndex : size_t
{
  PX = 0,
  PY = 1,
  YAW = 2,
  VYAW = 3,
  SLIP = 4,
  V = 5
};

enum UIndex : size_t
{
  FD = 0,
  FB = 1,
  STEER = 2
};

class DoubleTrackPlanarModel final : public base_vehicle_model::BaseVehicleModel
{
public:
  typedef std::shared_ptr<DoubleTrackPlanarModel> SharedPtr;
  typedef std::unique_ptr<DoubleTrackPlanarModel> UniquePtr;

  DoubleTrackPlanarModel(
    base_vehicle_model::BaseVehicleModelConfig::SharedPtr base_config,
    DoubleTrackPlanarModelConfig::SharedPtr config);

  const DoubleTrackPlanarModelConfig & get_config() const;

  size_t nx() const override;
  size_t nu() const override;

  void dynamics_jacobian(const casadi::DMDict & in, casadi::DMDict & out) override;
  void add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in) override;
  void calc_lon_control(
    const casadi::DMDict & in, double & throttle,
    double & brake_kpa) const override;
  void calc_lat_control(const casadi::DMDict & in, double & steering_rad) const override;

private:
  void compile_dynamics();

  DoubleTrackPlanarModelConfig::SharedPtr config_ {};
  casadi::Function dynamics_gamma_y_;
  casadi::Function dynamics_jac_;
};
}  // namespace double_track_planar_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // DOUBLE_TRACK_PLANAR_MODEL__DOUBLE_TRACK_PLANAR_MODEL_HPP_
