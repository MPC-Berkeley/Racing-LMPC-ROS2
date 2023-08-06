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
  BaseVehicleModel::SharedPtr model,
  const bool & full_dynamics)
: config_(mpc_config), model_(model),
  scale_x_(casadi::DM::ones(model_->nx())),
  scale_u_(casadi::DM::ones(model_->nu())),
  g_to_f_(utils::global_to_frenet_function<casadi::MX>(config_->N)),
  norm_2_(utils::norm_2_function(config_->N)),
  align_yaw_(utils::align_yaw_function(config_->N)),
  align_abscissa_(utils::align_abscissa_function(config_->N)),
  opti_(casadi::Opti()),
  X_(opti_.variable(model_->nx(), config_->N)),
  U_(opti_.variable(model_->nu(), config_->N - 1)),
  dU_(opti_.variable(model_->nu(), config_->N - 1)),
  X_ref_(opti_.parameter(model_->nx(), config_->N)),
  U_ref_(opti_.parameter(model_->nu(), config_->N - 1)),
  T_ref_(opti_.parameter(1, config_->N - 1)),
  x_ic_(opti_.parameter(model_->nx(), 1)),
  u_ic_(opti_.parameter(model_->nu(), 1)),
  bound_left_(opti_.parameter(1, config_->N)),
  bound_right_(opti_.parameter(1, config_->N)),
  total_length_(opti_.parameter(1, 1)),
  curvatures_(opti_.parameter(1, config_->N)),
  vel_ref_(opti_.parameter(1, config_->N)),
  solved_(false),
  sol_()
{
  using casadi::MX;
  using casadi::Slice;

  // configure solver
  auto p_opts = casadi::Dict{
    {"expand", true},
    {"print_time", config_->verbose ? true : false},
  };
  if (config_->jit) {
    p_opts["jit"] = true;
    p_opts["jit_options"] = casadi::Dict{{"flags", "-Ofast"}};
  }
  const auto s_opts = casadi::Dict{
    {"max_cpu_time", config_->max_cpu_time},
    {"tol", config_->tol},
    {"print_level", config_->verbose ? 5 : 0},
    {"max_iter", static_cast<casadi_int>(config_->max_iter)}
  };
  opti_.solver("ipopt", p_opts, s_opts);
  auto cost = MX::zeros(1);

  // set up abscissa offsets
  // const auto P0 = X_ref_(XIndex::PX, Slice());
  // const auto X0 = MX::vertcat({P0, MX::zeros(model_->nx() - 1, config_->N)});

  // set up track boundary constraint
  bool enable_boundary_slack = static_cast<double>(config_->q_boundary) > 0.0;
  const auto PY = X_(XIndex::PY, Slice()) * scale_x_(XIndex::PY);
  const auto margin = config_->margin + model_->get_base_config().chassis_config->b / 2.0;
  if (enable_boundary_slack) {
    boundary_slack_ = opti_.variable(1, config_->N);
    opti_.subject_to(
      opti_.bounded(
        bound_right_ + margin - boundary_slack_, PY,
        bound_left_ - margin + boundary_slack_));
    opti_.subject_to(boundary_slack_ >= 0.0);
    for (casadi_int i = 0; i < config_->N; i++) {
      cost += MX::mtimes({boundary_slack_(i).T(), config_->q_boundary, boundary_slack_(i)});
    }
  } else {
    opti_.subject_to(opti_.bounded(bound_right_ + margin, PY, bound_left_ - margin));
  }

  // --- MPC stage cost ---
  const auto x0 = X_(Slice(), 0) * scale_x_;
  for (size_t i = 0; i < config_->N - 1; i++) {
    const auto xi = X_(Slice(), i) * scale_x_;
    const auto ui = U_(Slice(), i - 1) * scale_u_;
    const auto dui = dU_(Slice(), i - 1) * scale_u_;
    // xi start with 1 since x0 must equal to x_ic and there is nothing we can do about it
    // const auto d_px =
    // utils::align_abscissa<MX>(xi(XIndex::PX), x0(XIndex::PX), total_length_) - x0(XIndex::PX);
    const auto x_base = model_->to_base_state()(casadi::MXDict{{"x", xi}, {"u", ui}}).at("x_out");
    const auto dv = x_base(XIndex::VX) - vel_ref_(i);
    auto ref = MX::zeros(3);
    ref(2) = vel_ref_(i);
    const auto err = x_base(Slice(XIndex::PY, XIndex::VX + 1)) - ref;
    const auto Q = MX::diag(MX::vertcat({config_->q_contour, config_->q_heading, config_->q_vel}));
    cost += MX::mtimes({err.T(), Q, err});

    cost += MX::mtimes({ui.T(), config_->R, ui});
    cost += MX::mtimes({dui.T(), config_->R_d, dui});
  }
  const auto xN = X_(Slice(), config_->N - 1) * scale_x_;
  const auto uN = U_(Slice(), config_->N - 2) * scale_u_;
  const auto x_base_N = model_->to_base_state()(casadi::MXDict{{"x", xN}, {"u", uN}}).at("x_out");
  const auto dv = x_base_N(XIndex::VX) - vel_ref_(config_->N - 1);
  auto refN = MX::zeros(3);
  refN(2) = vel_ref_(config_->N - 1);
  const auto errN = x_base_N(Slice(XIndex::PY, XIndex::VX + 1)) - refN;
  const auto Q = 10.0 *
    MX::diag(MX::vertcat({config_->q_contour, config_->q_heading, config_->q_vel}));
  cost += MX::mtimes({errN.T(), Q, errN});

  opti_.minimize(cost);

  // --- model constraints ---
  for (size_t i = 0; i < config_->N - 1; i++) {
    const auto xi = X_(Slice(), i) * scale_x_;
    const auto xip1 = X_(Slice(), i + 1) * scale_x_;
    const auto ui = U_(Slice(), i) * scale_u_;
    const auto ti = T_ref_(i);
    const auto k = curvatures_(i);
    casadi::MXDict constraint_in = {
      {"x", xi},
      {"u", ui},
      {"xip1", xip1},
      {"t", ti},
      {"k", k},
      {"track_length", total_length_}
    };

    const auto dui = dU_(Slice(), i) * scale_u_;
    constraint_in["dui"] = dui;

    model_->add_nlp_constraints(opti_, constraint_in);

    // primal bounds
    opti_.subject_to(opti_.bounded(config_->x_min, xi, config_->x_max));
    opti_.subject_to(opti_.bounded(config_->u_min, ui, config_->u_max));

    // dynamics constraints
    // auto xip1_temp = casadi::MX(xip1);
    // if (model_->get_base_config().modeling_config->use_frenet) {
    //   xip1_temp(XIndex::PX) =
    //     lmpc::utils::align_abscissa<casadi::MX>(
    //     xip1_temp(XIndex::PX), xi(XIndex::PX),
    //     total_length_);
    // } else {
    //   xip1_temp(XIndex::YAW) =
    //     lmpc::utils::align_yaw<casadi::MX>(xip1_temp(XIndex::YAW), xi(XIndex::YAW));
    // }

    if (full_dynamics) {
      // use full dynamics for dynamics constraints
      const auto xip1_pred =
        model_->discrete_dynamics()({{"x", xi}, {"u", ui}, {"k", k}, {"dt", ti}}).at("xip1");
      opti_.subject_to(xip1_pred - xip1 == 0);
    } else {
      // or use linearlized dynamics for dynamics constraints
      const auto xi_ref = X_ref_(Slice(), i);
      const auto ui_ref = U_ref_(Slice(), i);
      // const auto xi_ref = x_ic_;
      // const auto ui_ref = u_ic_;
      const auto AB =
        model_->discrete_dynamics_jacobian()(
        {{"x", xi_ref}, {"u", ui_ref}, {"k", k},
          {"dt", ti}});
      const auto x_ref_p1 =
        model_->discrete_dynamics()({{"x", xi_ref}, {"u", ui_ref}, {"k", k}, {"dt", ti}}).at(
        "xip1");
      const auto & A = AB.at("A");
      const auto & B = AB.at("B");
      const auto & g = AB.at("g");
      opti_.subject_to(
        (xip1 - x_ref_p1) -
        (MX::mtimes(A, (xi - xi_ref)) + MX::mtimes(B, (ui - ui_ref))) == 0);
      // opti_.subject_to(xip1_temp - (MX::mtimes(A, xi) + MX::mtimes(B, ui) + g) == 0);
    }

    // control rate constraints
    MX uim1;
    if (i == 0) {
      uim1 = u_ic_;
    } else {
      uim1 = U_(Slice(), i - 1) * scale_u_;
    }
    opti_.subject_to(uim1 + dui * ti == ui);
  }

  // --- initial state constraint ---
  opti_.subject_to(x0 == x_ic_);
}

const RacingMPCConfig & RacingMPC::get_config() const
{
  return *config_.get();
}

void RacingMPC::solve(const casadi::DMDict & in, casadi::DMDict & out, casadi::Dict & stats)
{
  using casadi::DM;
  using casadi::MX;
  using casadi::Slice;

  const auto & total_length = in.at("total_length");
  const auto & x_ic = in.at("x_ic");
  const auto & u_ic = in.at("u_ic");
  auto X_ref = in.at("X_ref");
  X_ref(XIndex::PX, Slice()) = align_abscissa_(
    casadi::DMDict{{"abscissa_1", X_ref(XIndex::PX, Slice())},
      {"abscissa_2", DM::ones(1, config_->N) * x_ic(XIndex::PX)},
      {"total_distance", DM::ones(1, config_->N) * total_length}}).at("abscissa_1_aligned");
  const auto & U_ref = in.at("U_ref");
  const auto & bound_left = in.at("bound_left");
  const auto & bound_right = in.at("bound_right");
  const auto & curvatures = in.at("curvatures");
  const auto & vel_ref = in.at("vel_ref");

  // set up the offsets
  const auto P0 = X_ref(XIndex::PX, Slice());
  // const auto X0 = DM::vertcat({P0, DM::zeros(model_->nx() - 1, config_->N)});

  // if optimal reference is given, typically from last MPC solution,
  // initialize with this reference.
  if (in.count("X_optm_ref")) {
    auto X_optm_ref = in.at("X_optm_ref");
    X_optm_ref(XIndex::PX, Slice()) = align_abscissa_(
      casadi::DMDict{{"abscissa_1", X_optm_ref(XIndex::PX, Slice())},
        {"abscissa_2", DM::ones(1, config_->N) * x_ic(XIndex::PX)},
        {"total_distance", DM::ones(1, config_->N) * total_length}}).at("abscissa_1_aligned");
    const auto & U_optm_ref = in.at("U_optm_ref");
    const auto & T_optm_ref = in.at("T_optm_ref");
    const auto & dU_optm_ref = in.at("dU_optm_ref");
    opti_.set_initial(X_ * scale_x_, X_optm_ref);
    opti_.set_initial(U_ * scale_u_, U_optm_ref);
    opti_.set_initial(dU_ * scale_u_, dU_optm_ref);
    opti_.set_value(T_ref_, T_optm_ref);
    if (sol_) {
      const auto lam_g0 = sol_->value(opti_.lam_g());
      opti_.set_initial(opti_.lam_g(), lam_g0);
    }
  } else {
    // TODO(haoru): just initialize with X_ref if no optm ref given
    if (!sol_) {
      throw std::runtime_error("No warm start given and no previous solution found.");
    }
    opti_.set_value(T_ref_, in.at("T_ref"));
    opti_.set_initial(sol_->value_variables());
    const auto last_abscissa_optm = (sol_->value(X_) * scale_x_)(XIndex::PX, Slice());
    const auto total_lengths = DM::ones(1, config_->N) * total_length;
    // Rember to wrap the abscissa to the range [0, total_length]
    const auto this_abscissa_optm =
      align_abscissa_(
      casadi::DMDict{{"abscissa_1", last_abscissa_optm},
        {"abscissa_2", P0}, {"total_distance", total_lengths}}).at("abscissa_1_aligned");
    opti_.set_initial((X_ * scale_x_)(XIndex::PX, Slice()), this_abscissa_optm);
    const auto lam_g0 = sol_->value(opti_.lam_g());
    opti_.set_initial(opti_.lam_g(), lam_g0);
  }

  // starting state must match
  opti_.set_value(x_ic_, x_ic);
  opti_.set_value(u_ic_, u_ic);

  // initialize other parameters
  opti_.set_value(X_ref_, X_ref);
  opti_.set_value(U_ref_, U_ref);
  opti_.set_value(bound_left_, bound_left);
  opti_.set_value(bound_right_, bound_right);
  opti_.set_value(total_length_, total_length);
  opti_.set_value(curvatures_, curvatures);
  opti_.set_value(vel_ref_, vel_ref);

  // solve problem
  try {
    sol_ = std::make_shared<casadi::OptiSol>(opti_.solve_limited());
    // const auto sol = opti.solve();
    solved_ = true;
    out["X_optm"] = sol_->value(X_) * scale_x_;
    out["U_optm"] = sol_->value(U_) * scale_u_;
    out["dU_optm"] = sol_->value(dU_) * scale_u_;
    stats = sol_->stats();
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    // throw e;
    out["X_optm"] = opti_.debug().value(X_) * scale_x_;
    out["U_optm"] = opti_.debug().value(U_) * scale_u_;
    out["dU_optm"] = opti_.debug().value(dU_) * scale_u_;
    stats = opti_.stats();
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
  X_ref(XIndex::VX, Slice()) = DM::linspace(current_vel, target_vel, config_->N);
  X_ref(XIndex::VYAW, Slice()) = X_ref(XIndex::VX, Slice()) / Radii;

  for (size_t i = 0; i < config_->N - 1; i++) {
    // fill control force with 2nd Newton's law
    const auto v0 = X_ref(XIndex::VX, i);
    const auto v1 = X_ref(XIndex::VX, i + 1);
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

BaseVehicleModel & RacingMPC::get_model()
{
  return *model_;
}

const bool & RacingMPC::solved() const
{
  return solved_;
}
}  // namespace racing_mpc
}  // namespace mpc
}  // namespace lmpc
