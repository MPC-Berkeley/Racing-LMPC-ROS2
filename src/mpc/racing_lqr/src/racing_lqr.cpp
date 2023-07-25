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

#include <math.h>
#include <exception>
#include <vector>
#include <iostream>
#include <chrono>

#include "racing_lqr/racing_lqr.hpp"
#include "lmpc_utils/utils.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_lqr
{
RacingLQR::RacingLQR(
  RacingLQRConfig::SharedPtr mpc_config,
  SingleTrackPlanarModel::SharedPtr model)
: config_(mpc_config), model_(model),
  c2d_(utils::c2d_function(model_->nx(), model_->nu(), config_->dt)),
  rk4_(utils::rk4_function(model_->nx(), model_->nu(), model_->dynamics()))
{
}

const RacingLQRConfig & RacingLQR::get_config() const
{
  return *config_.get();
}

void RacingLQR::solve(const casadi::DMDict & in, casadi::DMDict & out)
{
  using casadi::DM;
  using casadi::MX;
  using casadi::Slice;

  const auto & x_ic = in.at("x_ic");
  const auto & X_ref = in.at("X_ref");
  const auto & U_ref = in.at("U_ref");

  auto P = casadi::DMVector(config_->N, config_->Qf);
  auto K = casadi::DMVector(config_->N - 1, DM::zeros(model_->nu(), model_->nx()));
  auto As = casadi::DMVector(config_->N - 1, DM::zeros(model_->nx(), model_->nx()));
  auto Bs = casadi::DMVector(config_->N - 1, DM::zeros(model_->nx(), model_->nu()));
  for (int k = config_->N - 2; k >= 0; k--) {
    // obtain linearlized continuous dynamics
    const auto dyn_jac = model_->dynamics_jacobian()(
      casadi::DMDict{{"x", X_ref(Slice(), k)}, {"u", U_ref(Slice(), k)}});
    const auto & Ac = dyn_jac.at("A");
    const auto & Bc = dyn_jac.at("B");

    // convert continuous dynamics to discrete
    const auto dyn_d = c2d_(casadi::DMDict{{"Ac", Ac}, {"Bc", Bc}});
    As[k] = dyn_d.at("A");
    Bs[k] = dyn_d.at("B");

    // Ricatti
    K[k] =
      DM::solve(
      config_->R + DM::mtimes({Bs[k].T(), P[k + 1], Bs[k]}),
      DM::mtimes({Bs[k].T(), P[k + 1], As[k]}));
    P[k] = config_->Q + DM::mtimes({As[k].T(), P[k + 1], As[k] - DM::mtimes(Bs[k], K[k])});
  }

  // simulate
  auto X_optm = DM::zeros(model_->nx(), config_->N);
  X_optm(Slice(), 0) = x_ic;
  auto U_optm = DM::zeros(model_->nu(), config_->N - 1);
  for (size_t k = 0; k < config_->N - 1; k++) {
    U_optm(Slice(), k) =
      U_ref(Slice(), k) - DM::mtimes(K[k], X_optm(Slice(), k) - X_ref(Slice(), k));
    // TODO(haoru): add frenet support
    X_optm(Slice(), k + 1) = rk4_(
      casadi::DMDict{{"x", X_optm(Slice(), k)}, {"u", U_optm(
            Slice(), k)}, {"dt", config_->dt}, {"k", 0.0}}).at("xip1");
  }

  // calculate control
  out["u"] = U_optm(Slice(), 0);
  out["U_optm"] = U_optm;
  out["X_optm"] = X_optm;
}

SingleTrackPlanarModel & RacingLQR::get_model()
{
  return *model_;
}
}  // namespace racing_lqr
}  // namespace mpc
}  // namespace lmpc
