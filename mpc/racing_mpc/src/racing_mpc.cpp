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

#include "racing_mpc/racing_mpc.hpp"
#include "lmpc_utils/utils.hpp"

namespace lmpc
{
namespace mpc
{
namespace racing_mpc
{
RacingMPC::RacingMPC(
  RacingMPCConfig::SharedPtr mpc_config,
  SingleTrackPlanarModel::SharedPtr model)
: config_(mpc_config), model_(model),
  scale_x_({
        config_->average_track_width,
        config_->average_track_width,
        3.14, 1.0, 0.5,
        static_cast<double>(mpc_config->x_max(XIndex::V))
      }),
  scale_u_({
        abs(model_->get_config().Fd_max),
        abs(model_->get_config().Fb_max),
        abs(model_->get_base_config().steer_config->max_steer)
      }),
  g_to_f_(utils::global_to_frenet_function<casadi::MX>(config_->N)),
  norm_2_(utils::norm_2_function(config_->N)),
  opti_(casadi::Opti()),
  X_(opti_.variable(model_->nx(), config_->N)),
  U_(opti_.variable(model_->nu(), config_->N - 1)),
  X_ref_(opti_.parameter(model_->nx(), config_->N)),
  U_ref_(opti_.parameter(model_->nu(), config_->N - 1)),
  T_ref_(opti_.parameter(1, config_->N - 1)),
  x_ic_(opti_.parameter(model_->nx(), 1)),
  bound_left_(opti_.parameter(2, config_->N)),
  bound_right_(opti_.parameter(2, config_->N))
{
  using casadi::MX;
  using casadi::Slice;

  // configure solver
  const auto p_opts = casadi::Dict{
    {"expand", true},
    {"print_time", config_->verbose ? true : false}
  };
  const auto s_opts = casadi::Dict{
    {"max_cpu_time", config_->max_cpu_time},
    {"tol", config_->tol},
    {"constr_viol_tol", config_->constr_viol_tol},
    {"print_level", config_->verbose ? 5 : 0},
    {"max_iter", 500}
  };
  opti_.solver("ipopt", p_opts, s_opts);

  // set up problem
  const auto P0 = X_ref_(Slice(0, 2), Slice());
  const auto X0 = MX::vertcat({P0, MX::zeros(model_->nx() - 2, config_->N)});
  const auto Yaws = X_ref_(XIndex::YAW, Slice());

  // --- trajectory tracking cost function ---
  auto cost = MX::zeros(1);
  for (size_t i = 0; i < config_->N - 1; i++) {
    const auto xi = X_(Slice(), i) * scale_x_ + X0(Slice(), i);
    const auto ui = U_(Slice(), i) * scale_u_;
    const auto dx = xi - X_ref_(Slice(), i);
    const auto du = ui - U_ref_(Slice(), i);
    cost += 0.5 * MX::mtimes({dx.T(), config_->Q, dx}) + 0.5 * MX::mtimes({du.T(), config_->R, du});
  }
  const auto dxN = X_(Slice(), -1) * scale_x_ + X0(Slice(), -1) - X_ref_(Slice(), -1);
  cost += 0.5 * MX::mtimes({dxN.T(), config_->Qf, dxN});
  opti_.minimize(cost);

  // --- boundary constraints in frenet frame ---
  const auto P = X_(Slice(0, 2), Slice()) * scale_x_(Slice(0, 2));
  const auto Pf = g_to_f_({P, MX::zeros(2, config_->N), Yaws})[0];
  // opti.subject_to(Pf(0, Slice()) == 0.0);
  const auto margin = config_->margin + model_->get_base_config().chassis_config->b / 2.0;
  const auto dl = norm_2_(bound_left_ - P0)[0];
  const auto dr = norm_2_(bound_right_ - P0)[0] * -1.0;
  opti_.subject_to(opti_.bounded(dr + margin, Pf(1, Slice()), dl - margin));

  // --- model constraints ---
  for (size_t i = 0; i < config_->N - 1; i++) {
    const auto xi = X_(Slice(), i) * scale_x_ + X0(Slice(), i);
    const auto xip1 = X_(Slice(), i + 1) * scale_x_ + X0(Slice(), i + 1);
    const auto ui = U_(Slice(), i) * scale_u_;
    const auto ti = T_ref_(i);
    casadi::MXDict constraint_in = {
      {"x", xi},
      {"u", ui},
      {"xip1", xip1},
      {"t", ti}
    };
    if (i < config_->N - 2) {
      const auto uip1 = U_(Slice(), i + 1) * scale_u_;
      constraint_in["uip1"] = uip1;
    }
    model_->add_nlp_constraints(opti_, constraint_in);
  }

  // --- initial state constraint ---
  const auto x0 = X_(Slice(), 0) * scale_x_ + X0(Slice(), 0);
  opti_.subject_to(x0 == x_ic_);
}

const RacingMPCConfig & RacingMPC::get_config() const
{
  return *config_.get();
}

void RacingMPC::solve(const casadi::DMDict & in, casadi::DMDict & out)
{
  using casadi::DM;
  using casadi::MX;
  using casadi::Slice;

  const auto & X_ref = in.at("X_ref");
  const auto & U_ref = in.at("U_ref");
  const auto & x_ic = in.at("x_ic");
  const auto & bound_left = in.at("bound_left");
  const auto & bound_right = in.at("bound_right");
  const auto P0 = X_ref(Slice(0, 2), Slice());
  const auto X0 = DM::vertcat({P0, DM::zeros(model_->nx() - 2, config_->N)});

  // if optimal reference given, initialize with the reference
  if (in.count("X_optm_ref")) {
    const auto & X_optm_ref = in.at("X_optm_ref");
    const auto & U_optm_ref = in.at("U_optm_ref");
    const auto & T_optm_ref = in.at("T_optm_ref");
    opti_.set_initial(X_ * scale_x_, X_optm_ref - X0);
    opti_.set_initial(U_ * scale_u_, U_optm_ref);
    opti_.set_value(T_ref_, T_optm_ref);
  } else {
    opti_.set_value(T_ref_, in.at("T_ref"));
  }

  // starting state must match
  opti_.set_value(x_ic_, x_ic);

  // initialize other parameters
  opti_.set_value(X_ref_, X_ref);
  opti_.set_value(U_ref_, U_ref);
  opti_.set_value(bound_left_, bound_left);
  opti_.set_value(bound_right_, bound_right);

  // solve problem
  try {
    const auto sol = opti_.solve_limited();
    // const auto sol = opti.solve();
    out["X_optm"] = sol.value(X_) * scale_x_ + X0;
    out["U_optm"] = sol.value(U_) * scale_u_;

    opti_.set_initial(sol.value_variables());
    const auto lam_g0 = sol.value(opti_.lam_g());
    opti_.set_initial(opti_.lam_g(), lam_g0);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    // throw e;
    out["X_optm"] = opti_.debug().value(X_) * scale_x_ + X0;
    out["U_optm"] = opti_.debug().value(U_) * scale_u_;
  }
}

void RacingMPC::create_warm_start(const casadi::DMDict & in, casadi::DMDict & out)
{
  using casadi::DM;
  using casadi::Slice;

  const auto & P0 = in.at("P0");
  const auto & Yaws = in.at("Yaws");
  const auto & Radii = in.at("Radii");
  const auto & current_vel = in.at("current_vel");
  const auto & target_vel = in.at("target_vel");

  if (static_cast<size_t>(P0.size2()) != config_->N) {
    throw std::length_error("create_warm_start: P0 dimension does not match MPC dimension.");
  }
  if (static_cast<size_t>(Yaws.size2()) != config_->N) {
    throw std::length_error("create_warm_start: Yaws dimension does not match MPC dimension.");
  }
  if (static_cast<double>(current_vel) <= 0.0) {
    throw std::range_error("Current velocity cannot be smaller than or equal to zero.");
  }
  if (static_cast<double>(target_vel) <= 0.0) {
    throw std::range_error("Target velocity cannot be smaller than or equal to zero.");
  }

  auto X_ref = DM::zeros(model_->nx(), config_->N);
  auto U_ref = DM::zeros(model_->nu(), config_->N - 1);
  auto T_ref = DM::zeros(config_->N - 1, 1);

  X_ref(Slice(0, 2), Slice()) = P0;
  X_ref(XIndex::YAW, Slice()) = Yaws;
  X_ref(XIndex::V, Slice()) = DM::linspace(current_vel, target_vel, config_->N);
  X_ref(XIndex::V_YAW, Slice()) = X_ref(XIndex::V, Slice()) / Radii;

  for (size_t i = 0; i < config_->N - 1; i++) {
    // fill control force with 2nd Newton's law
    const auto v0 = X_ref(XIndex::V, i);
    const auto v1 = X_ref(XIndex::V, i + 1);
    const auto d = DM::norm_2(P0(Slice(), i) - P0(Slice(), i + 1));
    const auto a = static_cast<double>((pow(v1, 2) - pow(v0, 2)) / (2 * d));
    const auto f = model_->get_base_config().chassis_config->total_mass * a;
    if (f > 0.0) {
      U_ref(UIndex::FD, i) = f;
    } else {
      U_ref(UIndex::FB, i) = f;
    }
    T_ref(i) = d / v0;

    // fill steering angle with pure pursuit
    {
      U_ref(
        UIndex::STEER,
        i) = atan(model_->get_base_config().chassis_config->wheel_base / Radii(i));
    }
  }
  out["X_ref"] = X_ref;
  out["U_ref"] = U_ref;
}

const SingleTrackPlanarModel & RacingMPC::get_model() const
{
  return *model_;
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
