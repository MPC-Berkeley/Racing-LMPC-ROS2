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

void BaseVehicleModel::add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in)
{
  (void) opti;
  (void) in;
}
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
