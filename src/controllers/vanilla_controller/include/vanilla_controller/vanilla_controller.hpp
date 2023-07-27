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

#ifndef VANILLA_CONTROLLER__VANILLA_CONTROLLER_HPP_
#define VANILLA_CONTROLLER__VANILLA_CONTROLLER_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include <vehicle_model_factory/vehicle_model_factory.hpp>
#include <racing_trajectory/racing_trajectory.hpp>

#include "vanilla_controller/vanilla_controller_config.hpp"

namespace lmpc
{
namespace mpc
{
namespace vanilla_controller
{
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModelConfig;
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModel;
using lmpc::vehicle_model::base_vehicle_model::XIndex;
using lmpc::vehicle_model::base_vehicle_model::UIndex;
using lmpc::vehicle_model::racing_trajectory::RacingTrajectory;

class VanillaController
{
public:
  typedef std::shared_ptr<VanillaController> SharedPtr;
  typedef std::unique_ptr<VanillaController> UniquePtr;

  explicit VanillaController(
    VanillaControllerConfig::SharedPtr controller_config,
    BaseVehicleModel::SharedPtr model,
    RacingTrajectory::SharedPtr track);
  const VanillaControllerConfig & get_config() const;

  void solve(const casadi::DMDict & in, casadi::DMDict & out, casadi::Dict & stats);

  BaseVehicleModel & get_model();

protected:
  VanillaControllerConfig::SharedPtr config_ {};
  BaseVehicleModel::SharedPtr model_ {};
  RacingTrajectory::SharedPtr track_ {};
  utils::PidController pid_controller_;
};
}  // namespace vanilla_controller
}  // namespace mpc
}  // namespace lmpc
#endif  // VANILLA_CONTROLLER__VANILLA_CONTROLLER_HPP_
