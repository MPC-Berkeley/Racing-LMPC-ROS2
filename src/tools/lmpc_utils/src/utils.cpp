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

#include "lmpc_utils/utils.hpp"

namespace lmpc
{
namespace utils
{
casadi::Function align_yaw_function(const casadi_int & n)
{
  const auto yaw_1 = casadi::SX::sym("yaw_1", 1, 1);
  const auto yaw_2 = casadi::SX::sym("yaw_2", 1, 1);
  const auto yaw_1_aligned = align_yaw<casadi::SX>(yaw_1, yaw_2);
  return casadi::Function(
    "align_yaw", {yaw_1, yaw_2}, {yaw_1_aligned}, {"yaw_1", "yaw_2"},
    {"yaw_1_aligned"}).map(n);
}

casadi::Function align_abscissa_function(const casadi_int & n)
{
  const auto abscissa_1 = casadi::SX::sym("abscissa_1", 1, 1);
  const auto abscissa_2 = casadi::SX::sym("abscissa_2", 1, 1);
  const auto total_distance = casadi::SX::sym("total_distance", 1, 1);
  const auto abscissa_1_aligned =
    align_abscissa<casadi::SX>(abscissa_1, abscissa_2, total_distance);
  return casadi::Function(
    "align_abscissa", {abscissa_1, abscissa_2, total_distance}, {abscissa_1_aligned}, {"abscissa_1",
      "abscissa_2", "total_distance"},
    {"abscissa_1_aligned"}).map(n);
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

casadi::Function rk4_function(
  const casadi_int & nx, const casadi_int & nu, const double & dt,
  casadi::Function & dynamics)
{
  using casadi::SX;
  const auto x = SX::sym("x", nx, 1);
  const auto u = SX::sym("u", nu, 1);
  const auto k = SX::sym("k", 1, 1);

  const auto out1 = dynamics(casadi::SXDict{{"x", x}, {"u", u}, {"k", k}});
  const auto k1 = out1.at("x_dot");
  const auto out2 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k1}, {"u", u}, {"k", k}});
  const auto k2 = out2.at("x_dot");
  const auto out3 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k2}, {"u", u}, {"k", k}});
  const auto k3 = out3.at("x_dot");
  const auto out4 = dynamics(casadi::SXDict{{"x", x + dt * k3}, {"u", u}, {"k", k}});
  const auto k4 = out4.at("x_dot");
  const auto out = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  return casadi::Function("rk4", {x, u, k}, {out}, {"x", "u", "k"}, {"xip1"});
}

casadi::Function rk4_function(
  const casadi_int & nx, const casadi_int & nu,
  casadi::Function & dynamics)
{
  using casadi::SX;
  const auto x = SX::sym("x", nx, 1);
  const auto u = SX::sym("u", nu, 1);
  const auto dt = SX::sym("dt", 1, 1);
  const auto k = SX::sym("k", 1, 1);

  const auto out1 = dynamics(casadi::SXDict{{"x", x}, {"u", u}, {"k", k}});
  const auto k1 = out1.at("x_dot");
  const auto out2 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k1}, {"u", u}, {"k", k}});
  const auto k2 = out2.at("x_dot");
  const auto out3 = dynamics(casadi::SXDict{{"x", x + dt / 2.0 * k2}, {"u", u}, {"k", k}});
  const auto k3 = out3.at("x_dot");
  const auto out4 = dynamics(casadi::SXDict{{"x", x + dt * k3}, {"u", u}, {"k", k}});
  const auto k4 = out4.at("x_dot");
  const auto out = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  return casadi::Function("rk4", {x, u, k, dt}, {out}, {"x", "u", "k", "dt"}, {"xip1"});
}
}  // namespace utils
}  // namespace lmpc
