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

#include "single_track_planar_model/single_track_planar_model.hpp"
#include "lmpc_utils/utils.hpp"
#define GRAVITY 9.8

namespace lmpc
{
namespace vehicle_model
{
namespace single_track_planar_model
{
SingleTrackPlanarModel::SingleTrackPlanarModel(
  base_vehicle_model::BaseVehicleModelConfig::SharedPtr base_config,
  SingleTrackPlanarModelConfig::SharedPtr config)
: base_vehicle_model::BaseVehicleModel(base_config), config_(config)
{
  compile_dynamics();
}

const SingleTrackPlanarModelConfig & SingleTrackPlanarModel::get_config() const
{
  return *config_.get();
}

size_t SingleTrackPlanarModel::nx() const
{
  return 6;
}

size_t SingleTrackPlanarModel::nu() const
{
  return 3;
}

void SingleTrackPlanarModel::forward_dynamics(const casadi::DMDict & in, casadi::DMDict & out)
{
  auto dyn_in = in;
  if (base_config_->modeling_config->use_frenet) {
    dyn_in["k"] = 0.0;
  }
  const auto dyn_out = dynamics_(in);
  out.insert(dyn_out.begin(), dyn_out.end());
}

void SingleTrackPlanarModel::dynamics_jacobian(const casadi::DMDict & in, casadi::DMDict & out)
{
  const auto jac = dynamics_jac_(in);
  out.insert(jac.begin(), jac.end());
}

void SingleTrackPlanarModel::add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in)
{
  const auto & x = in.at("x");
  const auto & u = in.at("u");
  const auto & xip1 = in.at("xip1");
  const auto & t = in.at("t");
  const auto k =
    base_config_->modeling_config->use_frenet ? in.at("k") : casadi::MX::sym("k", 1, 1);

  const auto v = casadi::MX::hypot(x(XIndex::VX), x(XIndex::VY));
  const auto & fd = u(UIndex::FD);
  const auto & fb = u(UIndex::FB);
  const auto & delta = u(UIndex::STEER);

  const auto & delta_max = get_base_config().steer_config->max_steer;
  // const auto & mu = get_config().mu;
  const auto & P_max = get_config().P_max;
  const auto & Fd_max = get_config().Fd_max;
  const auto & Fb_max = get_config().Fb_max;
  const auto & Td = get_config().Td;
  const auto & Tb = get_config().Tb;
  const auto & Tdelta = get_base_config().steer_config->max_steer /
    get_base_config().steer_config->max_steer_rate;

  // dynamics constraint
  auto xip1_temp = casadi::MX(xip1);
  if (base_config_->modeling_config->use_frenet) {
    xip1_temp(XIndex::PX) =
      lmpc::utils::align_abscissa<casadi::MX>(
      xip1_temp(XIndex::PX), x(XIndex::PX),
      in.at("track_length"));
  } else {
    xip1_temp(XIndex::YAW) =
      lmpc::utils::align_yaw<casadi::MX>(xip1_temp(XIndex::YAW), x(XIndex::YAW));
  }

  const auto out1 = dynamics_({{"x", x}, {"u", u}, {"k", k}});
  const auto k1 = out1.at("x_dot");
  const auto out2 = dynamics_({{"x", x + t / 2.0 * k1}, {"u", u}, {"k", k}});
  const auto k2 = out2.at("x_dot");
  const auto out3 = dynamics_({{"x", x + t / 2.0 * k2}, {"u", u}, {"k", k}});
  const auto k3 = out3.at("x_dot");
  const auto out4 = dynamics_({{"x", x + t * k3}, {"u", u}, {"k", k}});
  const auto k4 = out4.at("x_dot");
  opti.subject_to(x + t / 6 * (k1 + 2 * k2 + 2 * k3 + k4) - xip1_temp == 0);

  // tyre constraints
  // const auto Fx_ij = out1.at("Fx_ij");
  // const auto Fy_ij = out1.at("Fy_ij");
  // const auto Fz_ij = out1.at("Fz_ij");
  // for (int i = 0; i < 2; i++) {
  //   opti.subject_to(pow(Fx_ij(i) / (mu * Fz_ij(i)), 2) +
  //   pow(Fy_ij(i) / (mu * Fz_ij(i)), 2) <= 1);
  // }

  // static actuator cconstraint
  opti.subject_to(v * fd <= P_max);
  // opti.subject_to(v >= 0.0);
  opti.subject_to(opti.bounded(0.0, fd, Fd_max));
  opti.subject_to(opti.bounded(Fb_max, fb, 0.0));
  opti.subject_to(pow(fd * fb, 2) <= 1.0);
  opti.subject_to(opti.bounded(-1.0 * delta_max, delta, delta_max));

  // dynamic actuator constraint
  if (in.count("uip1")) {
    const auto & uip1 = in.at("uip1");
    opti.subject_to((uip1(UIndex::FD) - fd) / t <= Fd_max / Td);
    opti.subject_to((uip1(UIndex::FB) - fb) / t >= Fb_max / Tb);
    opti.subject_to(
      opti.bounded(
        -delta_max / Tdelta, (uip1(UIndex::STEER) - delta) / t,
        delta_max / Tdelta));
  }
}

void SingleTrackPlanarModel::calc_lon_control(
  const casadi::DMDict & in, double & throttle,
  double & brake_kpa) const
{
  const auto & u = in.at("u").get_elements();
  const auto & fd = u[UIndex::FD];
  const auto & fb = u[UIndex::FB];
  throttle = 0.0;
  brake_kpa = 0.0;
  if (abs(fd) > abs(fb)) {
    throttle = calc_throttle(fd);
  } else {
    brake_kpa = calc_brake(fb);
  }
}

void SingleTrackPlanarModel::calc_lat_control(
  const casadi::DMDict & in,
  double & steering_rad) const
{
  const auto & u = in.at("u").get_elements();
  steering_rad = u[UIndex::STEER];
}

void SingleTrackPlanarModel::compile_dynamics()
{
  using casadi::SX;

  const auto x = SX::sym("x", nx());
  const auto u = SX::sym("u", nu());
  const auto k = SX::sym("k", 1);  // curvature for frenet frame

  const auto & py = x(XIndex::PY);
  const auto & phi = x(XIndex::YAW);  // yaw
  const auto & omega = x(XIndex::VYAW);  // yaw rate
  const auto & vx = x(XIndex::VX);  // body frame longitudinal velocity
  const auto & vy = x(XIndex::VY);  // body frame lateral velocity
  // const auto beta = atan2(vy, vx);  // slip angle
  // const auto v = casadi::SX::hypot(vx, vy);  // velocity magnitude
  const auto & fd = u(UIndex::FD);  // drive force
  const auto & fb = u(UIndex::FB);  // brake forcce
  const auto & delta = u(UIndex::STEER);  // front wheel angle
  const auto v_sq = vx * vx + vy * vy;

  const auto & kd_f = get_base_config().powertrain_config->kd;
  const auto & kb_f = get_base_config().front_brake_config->bias;  // front brake force bias
  const auto & m = get_base_config().chassis_config->total_mass;  // mass of car
  const auto & Jzz = get_base_config().chassis_config->moi;  // MOI around z axis
  const auto & l = get_base_config().chassis_config->wheel_base;  // wheelbase
  const auto & lr = get_base_config().chassis_config->cg_ratio * l;  // cg to front axle
  const auto lf = l - lr;  // cg to rear axle
  // const auto & twf = get_base_config().chassis_config->tw_f;  // front track width
  // const auto & twr = get_base_config().chassis_config->tw_r;  // rear track width
  const auto & fr = get_base_config().chassis_config->fr;  // rolling resistance coefficient
  const auto & hcog = get_base_config().chassis_config->cg_height;  // center of gravity height
  const auto & cl_f = get_base_config().aero_config->cl_f;  // downforce coefficient at front
  const auto & cl_r = get_base_config().aero_config->cl_r;  // downforce coefficient at rear
  const auto & rho = get_base_config().aero_config->air_density;  // air density
  const auto & A = get_base_config().aero_config->frontal_area;  // frontal area
  const auto & cd = get_base_config().aero_config->drag_coeff;  // drag coefficient
  const auto & mu = get_config().mu;  // tyre - track friction coefficient

  // magic tyre parameters
  const auto & tyre_f = *get_base_config().front_tyre_config;
  const auto & Bf = tyre_f.pacejka_b;  // magic formula B - front
  const auto & Cf = tyre_f.pacejka_c;  // magic formula C - front
  const auto & Ef = tyre_f.pacejka_e;  // magic formula E - front
  const auto & Fz0_f = tyre_f.pacejka_fz0;  // magic formula Fz0 - front
  const auto & eps_f = tyre_f.pacejka_eps;  // extended magic formula epsilon - front
  const auto & tyre_r = *get_base_config().rear_tyre_config;
  const auto & Br = tyre_r.pacejka_b;  // magic formula B - rear
  const auto & Cr = tyre_r.pacejka_c;  // magic formula C - rear
  const auto & Er = tyre_r.pacejka_e;  // magic formula E - rear
  const auto & Fz0_r = tyre_r.pacejka_fz0;  // magic formula Fz0 - rear
  const auto & eps_r = tyre_r.pacejka_eps;  // extended magic formula epsilon - rear

  // longitudinal tyre force Fx (eq. 4a, 4b)
  // TODO(haoru): consider differential
  const auto Fx_f = 0.5 * kd_f * fd + 0.5 * kb_f * fb - 0.5 * fr * m * GRAVITY * lr / l;
  const auto Fx_fl = Fx_f;
  // const auto Fx_fr = Fx_f;
  const auto Fx_r = 0.5 * (1 - kd_f) * fd + 0.5 * (1.0 - kb_f) * fb - 0.5 * fr * m * GRAVITY * lf /
    l;
  const auto Fx_rl = Fx_r;
  // const auto Fx_rr = Fx_r;

  // longitudinal acceleration (eq. 9)
  const auto ax = (fd + fb - 0.5 * cd * A * v_sq - fr * m * GRAVITY) / m;

  // vertical tyre force Fz (eq. 7a, 7b)
  const auto Fz_f = 0.5 * m * GRAVITY * lr / (lf + lr) - 0.5 * hcog / (lf + lr) * m * ax + 0.25 *
    cl_f * rho * A * v_sq;
  const auto Fz_fl = Fz_f;
  // const auto Fz_fr = Fz_f;
  const auto Fz_r = 0.5 * m * GRAVITY * lr / (lf + lr) + 0.5 * hcog / (lf + lr) * m * ax + 0.25 *
    cl_r * rho * A * v_sq;
  const auto Fz_rl = Fz_r;
  // const auto Fz_rr = Fz_r;

  // tyre sideslip angles alpha (eq. 6a, 6b)
  const auto a_fl = delta -
    atan((lf * omega + vy) / vx);
  // const auto a_fr = a_fl;
  const auto a_rl = atan((lr * omega - vy) / vx);
  // const auto a_rr = a_rl;

  // lateral tyre force Fy (eq. 5)
  const auto Fy_fl = mu * Fz_fl * (1.0 + eps_f * Fz_fl / Fz0_f) *
    sin(Cf * atan(Bf * a_fl - Ef * (Bf * a_fl - atan(Bf * a_fl))));
  // const auto Fy_fr = mu * Fz_fr * (1.0 + eps_f * Fz_fr / Fz0_f) *
  //   sin(Cf * atan(Bf * a_fr - Ef * (Bf * a_fr - atan(Bf * a_fr))));
  const auto Fy_rl = mu * Fz_rl * (1.0 + eps_r * Fz_rl / Fz0_r) *
    sin(Cr * atan(Br * a_rl - Er * (Br * a_rl - atan(Br * a_rl))));
  // const auto Fy_rr = mu * Fz_rr * (1.0 + eps_r * Fz_rr / Fz0_r) *
  //   sin(Cr * atan(Br * a_rr - Er * (Br * a_rr - atan(Br * a_rr))));

  // dynamics (eq. 3a, 3b, 3c)
  // const auto v_dot = 1.0 / m *
  //   ((Fx_rl + Fx_rr) * cos(beta) + (Fx_fl + Fx_fr) * cos(delta - beta) + (Fy_rl + Fy_rr) *
  //   sin(beta) - (Fy_fl + Fy_fr) * sin(delta - beta) - 0.5 * cd * rho * A * v_sq * cos(beta));
  // const auto beta_dot = -omega + 1.0 / (m * v) *
  //   (-(Fx_rl + Fx_rr) * sin(beta) + (Fx_fl + Fx_fr) * sin(delta - beta) + (Fy_rl + Fy_rr) * cos(
  //     beta) + (Fy_fl + Fy_fr) * cos(delta - beta) + 0.5 * cd * rho * A * v_sq * sin(beta));
  const auto omega_dot = 1.0 / Jzz *
    (-(2 * Fy_rl) * lr + ((2 * Fy_fl) * cos(delta) + (2 * Fx_fl) * sin(delta)) * lf);

  // dynamics (modified)
  // body frame acceleration vx_dot, vy_dot
  const auto vx_dot = 1.0 / m *
    ((2 * Fx_rl) + (2 * Fx_fl) * cos(delta) - (2 * Fy_fl) * sin(delta) -
    0.5 * cd * rho * A * v_sq) + omega * vy;
  const auto vy_dot = 1.0 / m *
    ((2 * Fy_rl) + (2 * Fy_fl) * cos(delta) + (2 * Fx_fl) * sin(delta)) -
    omega * vx;

  // global frame acceleration px_dot, py_dot
  auto px_dot = vx * cos(phi) - vy * sin(phi);
  auto py_dot = vx * sin(phi) + vy * cos(phi);
  auto phi_dot = omega;

  if (base_config_->modeling_config->use_frenet) {
    // convert to frenet frame
    px_dot /= (1 - py * k);
    phi_dot -= k * px_dot;
  }

  const auto x_dot = vertcat(px_dot, py_dot, phi_dot, vx_dot, vy_dot, omega_dot);
  const auto Fx_ij = vertcat(Fx_fl, Fx_rl);
  const auto Fy_ij = vertcat(Fy_fl, Fy_rl);
  const auto Fz_ij = vertcat(Fz_fl, Fz_rl);

  dynamics_ = casadi::Function(
    "single_track_planar_model",
    {x, u, k},
    {x_dot, Fx_ij, Fy_ij, Fz_ij},
    {"x", "u", "k"},
    {"x_dot", "Fx_ij", "Fy_ij", "Fz_ij"});

  const auto Ac = SX::jacobian(x_dot, x);
  const auto Bc = SX::jacobian(x_dot, u);

  dynamics_jac_ = casadi::Function(
    "single_track_planar_model_jacobian",
    {x, u, k},
    {Ac, Bc},
    {"x", "u", "k"},
    {"A", "B"}
  );
}
}  // namespace single_track_planar_model
}  // namespace vehicle_model
}  // namespace lmpc
