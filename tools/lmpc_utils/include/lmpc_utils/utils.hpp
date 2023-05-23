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

#ifndef LMPC_UTILS__UTILS_HPP_
#define LMPC_UTILS__UTILS_HPP_

#include "casadi/casadi.hpp"

namespace lmpc
{
namespace utils
{
template<typename T>
T align_yaw(const T & yaw_1, const T & yaw_2)
{
  const auto k = fabs(yaw_2 - yaw_1) + M_PI;
  const auto l = k - fmod(fabs(yaw_2 - yaw_1) + M_PI, 2 * M_PI);
  return yaw_1 + l * sign(yaw_2 - yaw_1);
}

enum TyreIndex : size_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3
};
}  // namespace utils
}  // namespace lmpc

#endif  // LMPC_UTILS__UTILS_HPP_
