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
  DoubleTrackPlanarModel::SharedPtr model)
: config_(mpc_config), model_(model),
  scale_x_({
        config_->average_track_width,
        config_->average_track_width,
        3.14, 1.0, 0.5,
        static_cast<double>(mpc_config->x_max(XIndex::V))
      }),
  scale_u_({
        model_->get_config().Fd_max,
        model_->get_config().Fb_max,
        model_->get_base_config().steer_config->max_steer}),
  scale_gamma_y_(model_->get_base_config().chassis_config->total_mass * 50.0)
{
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
  const auto & x_g = in.at("x_g");
  const auto & bound_left = in.at("bound_left");
  const auto & bound_right = in.at("bound_right");
  const auto P0 = X_ref(Slice(0, 2), Slice());
  const auto X0 = DM::vertcat({P0, DM::zeros(model_->nx() - 2, config_->N)});
  const auto Yaws = X_ref(XIndex::YAW);
  const auto V = X_ref(XIndex::V);

  auto opti = casadi::Opti();
  auto X = opti.variable(model_->nx(), config_->N);
  auto U = opti.variable(model_->nu(), config_->N - 1);
  auto T = opti.variable(config_->N - 1);
  auto Gamma_y = opti.variable(config_->N - 1);

  // minimum time trajectory tracking cost
  auto cost = MX::zeros(1);
  for (int i = 0; i < config_->N - 1; i++) {
    const auto dx = (X(Slice(), i) - X_ref(Slice(), i)) * scale_x_;
    const auto du = (U(Slice(), i) - U_ref(Slice(), i)) * scale_u_;
    cost += T(i) *
      (0.5 *
      MX::mtimes({dx.T(), config_->Q, dx}) + 0.5 * MX::mtimes({du.T(), config_->R, du}));
  }
  const auto dxN = (X(Slice(), -1) - X_ref(Slice(), -1)) * scale_x_;
  const auto duN = (U(Slice(), -1) - U_ref(Slice(), -1)) * scale_u_;
  cost += 0.5 * MX::mtimes({dxN.T(), config_->Qf, dxN});
  opti.minimize(cost);

  for (int i = 0; i < config_->N - 1; i++) {
    const auto xi = X(Slice(), i) * scale_x_ + X0(Slice(), i);
    const auto xip1 = X(Slice(), i + 1) * scale_x_ + X0(Slice(), i + 1);
    const auto ui = U(Slice(), i) * scale_u_;
    const auto ti = T(i);
    const auto gamma_y = Gamma_y(i) * scale_gamma_y_;

    // boundary constraints in frenet frame
    const auto p = X(Slice(0, 2), i) * scale_x_(Slice(0, 2));
    const auto p0 = P0(Slice(), i);
    const auto p1 = P0(Slice(), i + 1);
    const auto pf = lmpc::utils::global_to_frenet<MX>(p, MX::zeros(2), Yaws(i));
    const auto dl = MX::norm_2(bound_left(Slice(), i) - p0);
    const auto dr = MX::norm_2(bound_right(Slice(), i) - p0) * -1.0;
    opti.subject_to(pf(0) == 0);
    // TODO(haoru): handle case when margin cannot be met
    opti.subject_to(opti.bounded(dr + config_->margin, pf(1), dl - config_->margin));

    // model constraints
    casadi::MXDict in = {
      {"x", xi},
      {"u", ui},
      {"gamma_y", gamma_y},
      {"xip1", xip1},
      {"t", ti}
    };
    if (i < config_->N - 2) {
      const auto uip1 = U(Slice(), i + 1);
      in["uip1"] = uip1;
    }
    model_->add_nlp_constraints(opti, in);

    // time must be positive
    opti.subject_to(ti >= 0.0);

    // set warm start
    opti.set_initial(
      X(Slice(), i) * scale_x_,
      X_ref(Slice(), i) - X0
    );
    opti.set_initial(ui, U_ref(Slice(), i));
    opti.set_initial(ti, DM::norm_2(p1 - p0) / X_ref(XIndex::V));
  }

  // Starting state must match
  const auto x0 = X(Slice(), 0) * scale_x_ + X0(Slice(), 0);
  opti.subject_to(x0 == x_ic);

  // Final position, orientation must match
  const auto xN = X(Slice(), -1) * scale_x_ + X0(Slice(), -1);
  opti.subject_to(xN(Slice(0, 3)) == x_g(Slice(0, 3)));
  // opti.subject_to(xN(XIndex::V) == x_g(XIndex::V));

  // configure solver
  const auto p_opts = casadi::Dict{
    {"expand", true}
  };
  const auto s_opts = casadi::Dict{
    {"max_wall_time", config_->max_wall_time},
    {"tol", config_->tol},
    {"constr_viol_tol", config_->constr_viol_tol},
    {"print_level", 0}
  };
  opti.solver("ipopt", p_opts, s_opts);

  // solve problem
  try {
    const auto sol = opti.solve_limited();
    out["X_optm"] = sol.value(X) * scale_x_ + X0;
    out["U_optm"] = sol.value(U) * scale_u_;
    out["T_optm"] = sol.value(T);
    out["Gamma_y_optm"] = sol.value(Gamma_y) * scale_gamma_y_;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
