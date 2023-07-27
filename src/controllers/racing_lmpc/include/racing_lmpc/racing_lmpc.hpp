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

#ifndef RACING_LMPC__RACING_LMPC_HPP_
#define RACING_LMPC__RACING_LMPC_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "racing_lmpc/racing_lmpc_config.hpp"
#include "vehicle_model_factory/vehicle_model_factory.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_lmpc
{
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModelConfig;
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModel;
using lmpc::vehicle_model::base_vehicle_model::XIndex;
using lmpc::vehicle_model::base_vehicle_model::UIndex;

class RacingLMPC
{
public:
  typedef std::shared_ptr<RacingLMPC> SharedPtr;
  typedef std::unique_ptr<RacingLMPC> UniquePtr;

  explicit RacingLMPC(
    RacingLMPCConfig::SharedPtr mpc_config,
    BaseVehicleModel::SharedPtr model);
  const RacingLMPCConfig & get_config() const;

  void solve(const casadi::DMDict & in, casadi::DMDict & out, casadi::Dict & stats);

  void create_warm_start(const casadi::DMDict & in, casadi::DMDict & out);

  BaseVehicleModel & get_model();

  const bool & solved() const;

protected:
  RacingLMPCConfig::SharedPtr config_ {};
  BaseVehicleModel::SharedPtr model_ {};

  casadi::DM scale_x_;
  casadi::DM scale_u_;
  casadi::Function g_to_f_;
  casadi::Function norm_2_;
  casadi::Function align_yaw_;
  casadi::Function align_abscissa_;
  casadi::Opti opti_;

  // optimization variables
  casadi::MX X_;  // all the states, scaled
  casadi::MX U_;  // all the inputs, scaled
  casadi::MX boundary_slack_;

  // optimization parameters
  casadi::MX X_ref_;  // reference states, unscaled
  casadi::MX U_ref_;  // reference inputs, unscaled
  casadi::MX T_ref_;  // reference time, unscaled
  casadi::MX x_ic_;  // initial state, unscaled
  casadi::MX u_ic_;  // initial input, unscaled
  casadi::MX bound_left_;
  casadi::MX bound_right_;
  casadi::MX total_length_;
  casadi::MX curvatures_;
  casadi::MX vel_ref_;

  // flag if the nlp has been solved at least once
  bool solved_;
  std::shared_ptr<casadi::OptiSol> sol_;
};
}  // namespace racing_lmpc
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_LMPC__RACING_LMPC_HPP_
