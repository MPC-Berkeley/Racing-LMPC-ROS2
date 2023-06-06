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

#ifndef RACING_LQR__RACING_LQR_HPP_
#define RACING_LQR__RACING_LQR_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "racing_lqr/racing_lqr_config.hpp"
#include "single_track_planar_model/single_track_planar_model.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_lqr
{
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModelConfig;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModelConfig; \
  using lmpc::vehicle_model::single_track_planar_model::XIndex;
using lmpc::vehicle_model::single_track_planar_model::UIndex;

class RacingLQR
{
public:
  typedef std::shared_ptr<RacingLQR> SharedPtr;
  typedef std::unique_ptr<RacingLQR> UniquePtr;

  explicit RacingLQR(
    RacingLQRConfig::SharedPtr mpc_config,
    SingleTrackPlanarModel::SharedPtr model);
  const RacingLQRConfig & get_config() const;

  void solve(const casadi::DMDict & in, casadi::DMDict & out);

  const SingleTrackPlanarModel & get_model() const;

protected:
  RacingLQRConfig::SharedPtr config_ {};
  SingleTrackPlanarModel::SharedPtr model_ {};
  casadi::Function c2d_;
  casadi::Function rk4_;
};
}  // namespace racing_lqr
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_LQR__RACING_LQR_HPP_
