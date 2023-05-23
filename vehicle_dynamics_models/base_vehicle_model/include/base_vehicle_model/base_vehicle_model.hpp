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

#ifndef BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_HPP_
#define BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "base_vehicle_model/base_vehicle_model_config.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace base_vehicle_model
{
class BaseVehicleModel
{
public:
  typedef std::shared_ptr<BaseVehicleModel> SharedPtr;
  typedef std::unique_ptr<BaseVehicleModel> UniquePtr;

  explicit BaseVehicleModel(BaseVehicleModelConfig::SharedPtr config);

  void set_base_config(BaseVehicleModelConfig::SharedPtr config);
  const BaseVehicleModelConfig & get_base_config() const;

  virtual size_t nx() const;
  virtual size_t nu() const;

  virtual void forward_dynamics(const casadi::MXDict & in, casadi::MXDict & out);
  virtual void add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in);

protected:
  BaseVehicleModelConfig::SharedPtr base_config_ {};
  casadi::DM x_;
  casadi::DM u_;
};
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_HPP_
