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

casadi::Function align_yaw_function(const casadi_int & n)
{
  const auto yaw_1 = casadi::SX::sym("yaw_1", 1, 1);
  const auto yaw_2 = casadi::SX::sym("yaw_2", 1, 1);
  const auto yaw_1_aligned = align_yaw<casadi::SX>(yaw_1, yaw_2);
  return casadi::Function(
    "align_yaw", {yaw_1, yaw_2}, {yaw_1_aligned}, {"yaw_1", "yaw_2"},
    {"yaw_1_aligned"}).map(n);
}

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

casadi::Function norm_2_function(const casadi_int & n)
{
  const auto p = casadi::MX::sym("p", 2, 1);
  const auto out = casadi::MX::norm_2(p);
  return casadi::Function("norm_2", {p}, {out}).map(n);
}

casadi::Function c2d_function(const casadi_int & nx, const casadi_int & nu, const double & dt)
{
  using casadi::MX;
  using casadi::Slice;
  const auto Ac = MX::sym("Ac", nx, nx);
  const auto Bc = MX::sym("Bc", nx, nu);
  auto M = MX::zeros(nx + nu, nx + nu);
  M(Slice(0, nx), Slice(0, nx)) = Ac;
  M(Slice(0, nx), Slice(nx, nx + nu)) = Bc;
  const auto exp_M = MX::expm(M * dt);
  const auto A = exp_M(Slice(0, nx), Slice(0, nx));
  const auto B = exp_M(Slice(0, nx), Slice(nx, nx + nu));
  return casadi::Function("c2d", {Ac, Bc}, {A, B}, {"Ac", "Bc"}, {"A", "B"});
}

/**
 * @brief Create a RK4 integrator whith fixed dt
 *
 * @param nx size of state
 * @param nu size of control
 * @param dt time step
 * @param dynamics continuious dynamics function
 * @return casadi::Function with inputs `x`, `u` and outputs next state `xip1`.
 */
casadi::Function rk4_function(
  const casadi_int & nx, const casadi_int & nu, const double & dt,
  casadi::Function & dynamics)
{
  using casadi::SX;
  const auto x = SX::sym("x", nx, 1);
  const auto u = SX::sym("u", nu, 1);

  const auto out1 = dynamics(casadi::SXDict{{"x", x}, {"u", u}});
  const auto k1 = out1.at("x_dot");
  const auto out2 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k1}, {"u", u}});
  const auto k2 = out2.at("x_dot");
  const auto out3 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k2}, {"u", u}});
  const auto k3 = out3.at("x_dot");
  const auto out4 = dynamics(casadi::SXDict{{"x", x + dt * k3}, {"u", u}});
  const auto k4 = out4.at("x_dot");
  const auto out = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  return casadi::Function("rk4", {x, u}, {out}, {"x", "u"}, {"xip1"});
}

/**
 * @brief Create a RK4 integrator whith dt as an input
 *
 * @param nx size of state
 * @param nu size of control
 * @param dynamics continuious dynamics function
 * @return casadi::Function with inputs `x`, `u` and `dt` and outputs next state `xip1`.
 */
casadi::Function rk4_function(
  const casadi_int & nx, const casadi_int & nu,
  casadi::Function & dynamics)
{
  using casadi::SX;
  const auto x = SX::sym("x", nx, 1);
  const auto u = SX::sym("u", nu, 1);
  const auto dt = SX::sym("dt", 1, 1);

  const auto out1 = dynamics(casadi::SXDict{{"x", x}, {"u", u}});
  const auto k1 = out1.at("x_dot");
  const auto out2 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k1}, {"u", u}});
  const auto k2 = out2.at("x_dot");
  const auto out3 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k2}, {"u", u}});
  const auto k3 = out3.at("x_dot");
  const auto out4 = dynamics(casadi::SXDict{{"x", x + dt * k3}, {"u", u}});
  const auto k4 = out4.at("x_dot");
  const auto out = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  return casadi::Function("rk4", {x, u, dt}, {out}, {"x", "u", "dt"}, {"xip1"});
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
