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

#ifndef VEHICLE_MODEL_FACTORY__VEHICLE_MODEL_FACTORY_HPP_
#define VEHICLE_MODEL_FACTORY__VEHICLE_MODEL_FACTORY_HPP_

#include <memory>
#include <string>

#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>

#include "base_vehicle_model/base_vehicle_model.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace vehicle_model_factory
{
base_vehicle_model::BaseVehicleModel::SharedPtr load_vehicle_model(
  const std::string model_name,
  rclcpp::Node * node);
}  // namespace vehicle_model_factory
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // VEHICLE_MODEL_FACTORY__VEHICLE_MODEL_FACTORY_HPP_
