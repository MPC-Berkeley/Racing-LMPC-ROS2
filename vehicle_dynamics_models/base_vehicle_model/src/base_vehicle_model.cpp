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
: base_config_(config), dynamics_(compile_dynamics())
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

casadi::Function & BaseVehicleModel::dynamics()
{
  return dynamics_;
}

size_t BaseVehicleModel::nx() const
{
  return 1;
}

size_t BaseVehicleModel::nu() const
{
  return 1;
}

casadi::Function BaseVehicleModel::compile_dynamics()
{
  auto x = casadi::SX::sym("x", nx(), 1);
  auto u = casadi::SX::sym("u", nu(), 1);
  auto x_dot = casadi::SX::zeros(1, 1);
  return casadi::Function("base_dynamics", {x, u}, {x_dot});
}
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
