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

#include "racing_trajectory/racing_trajectory.hpp"
#include "lmpc_utils/utils.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{
RacingTrajectory::RacingTrajectory(const casadi::DM & traj)
: traj_(traj),
  abscissa_(traj_(TrajectoryIndex::DIST_TO_SF_BWD, casadi::Slice())),
  norm_2_(utils::norm_2_function(traj_.size2())),
  total_length_(traj_(TrajectoryIndex::DIST_TO_SF_FWD, 0)),
  kd_tree_(traj_(TrajectoryIndex::PX, casadi::Slice()).get_elements(),
    traj_(TrajectoryIndex::PY, casadi::Slice()).get_elements())
{
  using casadi::Slice;
  using casadi::DM;
  using casadi::MX;
  using casadi::Function;

  {
    // create the interpolants
    auto interpolants = DM(traj_);
    // cubic spline interpolation requires 3 points on both ends to
    // garantee smoothness and interpolatability at the ends.
    // lets repeat the first 3 points at the end and the last 3 points at the beginning.
    // note that we are actually appending the first 4 points to the end
    // in order to close the trajectory loop.

    // append the first 4 points
    interpolants = DM::horzcat({interpolants, interpolants(Slice(), Slice(0, 4))});

    // make sure the distance measure is continuous
    interpolants(
      TrajectoryIndex::DIST_TO_SF_BWD,
      Slice(-4, std::numeric_limits<casadi_int>::max())) += total_length_;

    // prepend the last 3 points
    interpolants = DM::horzcat({interpolants(Slice(), Slice(-7, -4)), interpolants});

    // make sure the distance measure is continuous
    interpolants(TrajectoryIndex::DIST_TO_SF_BWD, Slice(0, 3)) -= total_length_;

    // build the interpolation functions
    auto norm_2_interpolant = utils::norm_2_function(interpolants.size2());
    const auto abscissa = interpolants(TrajectoryIndex::DIST_TO_SF_BWD, Slice());
    const auto t_left =
      norm_2_interpolant(
      interpolants(
        Slice(TrajectoryIndex::PX, TrajectoryIndex::PY + 1),
        Slice()) -
      interpolants(
        Slice(TrajectoryIndex::LEFT_BOUND_X, TrajectoryIndex::LEFT_BOUND_Y + 1),
        Slice()))[0];
    const auto t_right = -1.0 *
      norm_2_interpolant(
      interpolants(
        Slice(TrajectoryIndex::PX, TrajectoryIndex::PY + 1),
        Slice()) -
      interpolants(
        Slice(TrajectoryIndex::RIGHT_BOUND_X, TrajectoryIndex::RIGHT_BOUND_Y + 1),
        Slice()))[0];
    const auto left_intp = casadi::interpolant(
      "left_intp_impl", "bspline",
      {abscissa.get_elements()}, t_left.get_elements());
    const auto right_intp = casadi::interpolant(
      "right_intp_impl", "bspline",
      {abscissa.get_elements()}, t_right.get_elements());
    const auto x_intp = casadi::interpolant(
      "x_intp_impl", "bspline",
      {abscissa.get_elements()}, interpolants(TrajectoryIndex::PX, Slice()).get_elements());
    const auto y_intp = casadi::interpolant(
      "y_intp_impl", "bspline",
      {abscissa.get_elements()}, interpolants(TrajectoryIndex::PY, Slice()).get_elements());
    const auto vel_intp = casadi::interpolant(
      "vel_intp_impl", "bspline",
      {abscissa.get_elements()}, interpolants(TrajectoryIndex::SPEED, Slice()).get_elements());

    const auto s = MX::sym("s", 1, 1);
    const auto s_mod = utils::align_abscissa<MX>(s, total_length_ / 2.0, total_length_);
    const auto s_mod_sym = MX::sym("s_mod", 1, 1);

    MX dx, dy;
    MX d2x = MX::hessian(x_intp(s_mod_sym)[0], s_mod_sym, dx);
    MX d2y = MX::hessian(y_intp(s_mod_sym)[0], s_mod_sym, dy);
    const auto d2x_func = Function("d2x", {s_mod_sym}, {d2x});
    const auto d2y_func = Function("d2y", {s_mod_sym}, {d2y});
    const auto dx_func = Function("dx", {s_mod_sym}, {dx});
    const auto dy_func = Function("dy", {s_mod_sym}, {dy});
    const auto yaw = MX::atan2(dy_func(s_mod)[0], dx_func(s_mod)[0]);
    const auto curvature =
      abs(dx_func(s_mod)[0] * d2y_func(s_mod)[0] - dy_func(s_mod)[0] * d2x_func(s_mod)[0]) /
      MX::sqrt(MX::pow(MX::pow(dx_func(s_mod)[0], 2) + MX::pow(dy_func(s_mod)[0], 2), 3));

    yaw_intp_ = Function("yaw_intp", {s}, {yaw});
    curvature_intp_ = Function("curvature_intp", {s}, {curvature});
    left_intp_ = Function("left_intp", {s}, {left_intp(s_mod)});
    right_intp_ = Function("right_intp", {s}, {right_intp(s_mod)});
    x_intp_ = Function("x_intp", {s}, {x_intp(s_mod)});
    y_intp_ = Function("y_intp", {s}, {y_intp(s_mod)});
    vel_intp_ = Function("vel_intp", {s}, {vel_intp(s_mod)});
  }

  // build the frenet to global transformation
  {
    const auto s = MX::sym("s", 1, 1);
    const auto s_mod = utils::align_abscissa<MX>(s, total_length_ / 2.0, total_length_);
    const auto t = MX::sym("t", 1, 1);
    const auto xi = MX::sym("xi", 1, 1);
    const auto x0 = x_intp_(s_mod)[0];
    const auto y0 = y_intp_(s_mod)[0];
    const auto yaw0 = yaw_intp_(s_mod)[0];
    const auto d_x = -1.0 * sin(yaw0) * t;
    const auto d_y = cos(yaw0) * t;
    const auto phi = utils::align_yaw<MX>(yaw0 + xi, 0.0);
    const auto out = MX::vertcat({x0 + d_x, y0 + d_y, phi});
    frenet_to_global_ = Function("frenet_to_global", {MX::vertcat({s, t, xi})}, {out});
  }

  // build the global to frenet transformation
  {
    const auto x = MX::sym("x", 1, 1);
    const auto y = MX::sym("y", 1, 1);
    const auto phi = MX::sym("phi", 1, 1);
    const auto s0 = MX::sym("s0", 1, 1);
    const auto s0_mod = utils::align_abscissa<MX>(s0, total_length_ / 2.0, total_length_);
    const auto t0 = MX::sym("t0", 1, 1);
    const auto s = MX::sym("s", 1, 1);
    const auto dist_sq = MX::sumsqr(
      frenet_to_global_(MX::vertcat({s, 0.0, 0.0}))[0](Slice(
        0,
        2)) - MX::vertcat(
        {x, y}));
    const auto qp = casadi::MXDict{{"x", s}, {"f", dist_sq}, {"p", MX::vertcat(
          {x, y})}};
    global_to_frenet_sol_ = casadi::nlpsol(
      "global_to_frenet_sol", "sqpmethod", qp,
          {
            {"print_time", false},
            {"print_header", false},
            {"print_iteration", false},
            {"print_status", false},
            {"qpsol", "qrqp"},
            {"qpsol_options", casadi::Dict
              {
                {"print_header", false}, {"print_iter", false}, {"print_info", false}
              }
            }
          }
    );
    const auto sol =
      global_to_frenet_sol_({{"x0", s0_mod}, {"p", MX::vertcat({x, y})}});
    auto s_out = sol.at("x");
    s_out = utils::align_abscissa<MX>(s_out, total_length_ / 2.0, total_length_);
    const auto x_out = x_intp_(s_out)[0];
    const auto y_out = y_intp_(s_out)[0];
    const auto yaw_out = yaw_intp_(s_out)[0];
    const auto t_out = MX::hypot(x - x_out, y - y_out) * lateral_sign<MX>(
      MX::vertcat(
        {x,
          y}), MX::vertcat(
        {x_out, y_out, yaw_out}));
    const auto xi_out = utils::align_yaw<MX>(phi, yaw_out) - yaw_out;
    global_to_frenet_ = Function(
      "global_to_frenet", {MX::vertcat(
          {x, y, phi, s0,
            t0})},
      {MX::vertcat({s_out, t_out, xi_out})});
  }
}

RacingTrajectory::RacingTrajectory(const std::string & file_name)
: RacingTrajectory(casadi::DM::from_file(file_name).T())
{
}

void RacingTrajectory::frenet_to_global(const FrenetPose2D & frenet_pose, Pose2D & global_pose)
{
  const auto out = frenet_to_global_(
    casadi::DM{frenet_pose.position.s, frenet_pose.position.t,
      frenet_pose.yaw})[0].get_elements();
  global_pose.position.x = out[0];
  global_pose.position.y = out[1];
  global_pose.yaw = out[2];
}

void RacingTrajectory::global_to_frenet(
  const Pose2D & global_pose, FrenetPose2D & frenet_pose,
  const bool & initialize_with_previous
)
{
  FrenetPose2D p0 = {0.0, 0.0, 0.0};
  if (initialize_with_previous) {
    p0 = frenet_pose;
  } else {
    // initialize with the closest point on the trajectory
    const size_t idx = kd_tree_.find_closest_waypoint_index(
      global_pose.position.x,
      global_pose.position.y);
    p0.position.s = static_cast<double>(abscissa_(idx));

    Pose2D p0_g;
    kd_tree_.get_waypoint(idx, p0_g.position.x, p0_g.position.y);
    p0_g.yaw = static_cast<double>(yaw_intp_(casadi::DM(p0.position.s))[0]);
    p0.position.t =
      static_cast<double>(distance(global_pose.position, p0_g.position)) * lateral_sign(
      global_pose.position, p0_g);
    p0.yaw = utils::align_yaw(global_pose.yaw, p0_g.yaw);
  }

  const auto out = global_to_frenet_(
    casadi::DM{
          global_pose.position.x, global_pose.position.y, global_pose.yaw,
          p0.position.s, p0.position.t
        })[0].get_elements();
  frenet_pose.position.s = out[0];
  frenet_pose.position.t = out[1];
  frenet_pose.yaw = out[2];
}

casadi::Function & RacingTrajectory::frenet_to_global_function()
{
  return frenet_to_global_;
}

casadi::Function & RacingTrajectory::global_to_frenet_function()
{
  return global_to_frenet_;
}

casadi::Function & RacingTrajectory::curvature_interpolation_function()
{
  return curvature_intp_;
}

casadi::Function & RacingTrajectory::left_boundary_interpolation_function()
{
  return left_intp_;
}

casadi::Function & RacingTrajectory::right_boundary_interpolation_function()
{
  return right_intp_;
}

casadi::Function & RacingTrajectory::x_interpolation_function()
{
  return x_intp_;
}

casadi::Function & RacingTrajectory::y_interpolation_function()
{
  return y_intp_;
}

casadi::Function & RacingTrajectory::yaw_interpolation_function()
{
  return yaw_intp_;
}

casadi::Function & RacingTrajectory::velocity_interpolation_function()
{
  return vel_intp_;
}

const double & RacingTrajectory::total_length() const
{
  return total_length_;
}
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
