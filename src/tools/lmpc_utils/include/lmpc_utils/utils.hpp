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
  const auto d_yaw = yaw_1 - yaw_2;
  const auto d_yaw_aligned = atan2(sin(d_yaw), cos(d_yaw));
  return d_yaw_aligned + yaw_2;
}

casadi::Function align_yaw_function(const casadi_int & n);

template<typename T>
T align_abscissa(const T & s1, const T & s2, const T & s_total)
{
  const auto k = fabs(s2 - s1) + s_total / 2.0;
  const auto l = k - fmod(fabs(s2 - s1) + s_total / 2.0, s_total);
  return s1 + l * sign(s2 - s1);
}

casadi::Function align_abscissa_function(const casadi_int & n);

template<typename T>
T global_to_frenet(const T & p, const T & p0, const T & yaw)
{
  using casadi::SX;
  using casadi::MX;
  using casadi::DM;
  const auto cos_theta = cos(-yaw);
  const auto sin_theta = sin(-yaw);
  const auto R = T(
    T::vertcat(
      {
        T::horzcat({cos_theta, -sin_theta}),
        T::horzcat({sin_theta, cos_theta})
      }));
  return T::mtimes(R, p - p0);
}

template<typename T>
casadi::Function global_to_frenet_function(const casadi_int & n)
{
  const auto p = T::sym("p", 2, 1);
  const auto p0 = T::sym("p0", 2, 1);
  const auto yaw = T::sym("yaw", 1, 1);
  const auto out = global_to_frenet<T>(p, p0, yaw);
  return casadi::Function("global_to_frenet", {p, p0, yaw}, {out}).map(n);
}

casadi::Function norm_2_function(const casadi_int & n);

casadi::Function c2d_function(const casadi_int & nx, const casadi_int & nu, const double & dt);

/**
 * @brief Create a RK4 integrator whith fixed dt
 *
 * @param nx size of state
 * @param nu size of control
 * @param dt time step
 * @param dynamics continuous dynamics function
 * @return casadi::Function with inputs `x`, `u` and outputs next state `xip1`.
 */
casadi::Function rk4_function(
  const casadi_int & nx, const casadi_int & nu, const double & dt,
  const casadi::Function & dynamics);

/**
 * @brief Create a RK4 integrator
 *
 * @param nx size of state
 * @param nu size of control
 * @param dynamics continuous dynamics function
 * @return casadi::Function with inputs `x`, `u` and `dt` and outputs next state `xip1`.
 */
casadi::Function rk4_function(
  const casadi_int & nx, const casadi_int & nu,
  const casadi::Function & dynamics);

/**
 * @brief Create a Euler integrator
 *
 * @param nx size of state
 * @param nu size of control
 * @param dynamics continuous dynamics function
 * @return casadi::Function with inputs `x`, `u` and `dt` and outputs next state `xip1`.
 */
casadi::Function euler_function(
  const casadi_int & nx, const casadi_int & nu,
  casadi::Function & dynamics);

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
