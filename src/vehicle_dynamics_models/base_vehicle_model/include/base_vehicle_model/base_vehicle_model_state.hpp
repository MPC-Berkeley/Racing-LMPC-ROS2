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

#ifndef BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_STATE_HPP_
#define BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_STATE_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "base_vehicle_model/base_vehicle_model_config.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace base_vehicle_model
{
struct BaseVehicleWheelSpeed
{
  double fl_radps = 0.0;
  double fr_radps = 0.0;
  double rl_radps = 0.0;
  double rr_radps = 0.0;
};

/**
 * @brief Anxillary state information commonly present in VD models.
 * They are also used in the throttle and brake calculation.
 *
 *
 */
struct BaseVehicleModelState
{
  BaseVehicleWheelSpeed wheel_speed;
  double engine_rpm = 0.0;
  uint8_t gear = 1;
};
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_STATE_HPP_
