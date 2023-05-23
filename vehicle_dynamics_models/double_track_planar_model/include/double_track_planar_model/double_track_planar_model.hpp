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

class DoubleTrackPlanarModel final : public base_vehicle_model::BaseVehicleModel
{
public:
  DoubleTrackPlanarModel(
    base_vehicle_model::BaseVehicleModelConfig::SharedPtr base_config,
    DoubleTrackPlanarModelConfig::SharedPtr config);

  const DoubleTrackPlanarModelConfig & get_config() const;

  size_t nx() const override;
  size_t nu() const override;

  void forward_dynamics(const casadi::MXDict & in, casadi::MXDict & out) override;
  void add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in) override;

private:
  void compile_dynamics();

  DoubleTrackPlanarModelConfig::SharedPtr config_ {};
  casadi::Function dynamics_;
  casadi::Function lateral_load_transfer_;
};
}  // namespace double_track_planar_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // DOUBLE_TRACK_PLANAR_MODEL__DOUBLE_TRACK_PLANAR_MODEL_HPP_
