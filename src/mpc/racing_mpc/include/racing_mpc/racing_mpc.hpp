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

#ifndef RACING_MPC__RACING_MPC_HPP_
#define RACING_MPC__RACING_MPC_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "racing_mpc/racing_mpc_config.hpp"
#include "vehicle_model_factory/vehicle_model_factory.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModelConfig;
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModel;
using lmpc::vehicle_model::base_vehicle_model::XIndex;
using lmpc::vehicle_model::base_vehicle_model::UIndex;

class RacingMPC
{
public:
  typedef std::shared_ptr<RacingMPC> SharedPtr;
  typedef std::unique_ptr<RacingMPC> UniquePtr;

  explicit RacingMPC(
    RacingMPCConfig::SharedPtr mpc_config,
    BaseVehicleModel::SharedPtr model);
  const RacingMPCConfig & get_config() const;

  void solve(const casadi::DMDict & in, casadi::DMDict & out);

  void create_warm_start(const casadi::DMDict & in, casadi::DMDict & out);

  BaseVehicleModel & get_model();

  const bool & solved() const;

protected:
  RacingMPCConfig::SharedPtr config_ {};
  BaseVehicleModel::SharedPtr model_ {};

  casadi::DM scale_x_;
  casadi::DM scale_u_;
  casadi::Function g_to_f_;
  casadi::Function norm_2_;
  casadi::Function align_yaw_;
  casadi::Function align_abscissa_;
  casadi::Opti opti_;

  // optimization variables
  casadi::MX X_;
  casadi::MX U_;
  casadi::MX boundary_slack_;

  // optimization parameters
  casadi::MX X_ref_;
  casadi::MX U_ref_;
  casadi::MX T_ref_;
  casadi::MX x_ic_;
  casadi::MX u_ic_;
  casadi::MX bound_left_;
  casadi::MX bound_right_;
  casadi::MX total_length_;
  casadi::MX curvatures_;
  casadi::MX vel_ref_;

  // flag if the nlp has been solved at least once
  bool solved_;
  std::shared_ptr<casadi::OptiSol> sol_;
};
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_MPC__RACING_MPC_HPP_
