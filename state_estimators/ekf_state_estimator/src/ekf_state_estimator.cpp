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

#include "ekf_state_estimator/ekf_state_estimator.hpp"
#include "lmpc_utils/utils.hpp"

namespace lmpc
{
namespace state_estimator
{
namespace ekf_state_estimator
{
EKFStateEstimator::EKFStateEstimator(
  EKFStateEstimatorConfig::SharedPtr ekf_config,
  SingleTrackPlanarModel::SharedPtr model)
: config_(ekf_config), model_(model),
  rk4_(utils::rk4_function(model_->nx(), model_->nu(), model_->dynamics())),
  compiled_(false), hs_(), h_jacs_(), x_(model_->nx(), 1), u_(model_->nu(), 1),
  P_(config_->P0), K_(model_->nx(), 0)
{
  const auto x = casadi::SX::sym("x", model_->nx(), 1);
  const auto u = casadi::SX::sym("u", model_->nu(), 1);
  const auto xip1 = rk4_(casadi::SXDict{{"x", x}, {"u", u}}).at("xip1");
  const auto F = casadi::SX::jacobian(xip1, x);
  F_ = casadi::Function("discrete_dynamics_jacobian", {x, u}, {F}, {"x", "u"}, {"F"});
}

const EKFStateEstimatorConfig & EKFStateEstimator::get_config() const
{
  return *config_.get();
}

const SingleTrackPlanarModel & EKFStateEstimator::get_model() const
{
  return *model_;
}

const int64_t & EKFStateEstimator::get_latest_timestamp() const
{
  return nanosec_;
}

void EKFStateEstimator::register_observation(
  const std::string & name, const casadi_int & nz,
  casadi::Function & h)
{
  if (is_compiled()) {
    throw EKFAlreadyCompiledException();
  }

  if (hs_.count(name)) {
    throw ObservationNameAlreadyExistsException(name.c_str());
  }

  // store observation function
  hs_[name] = h;

  // create and store jacobian
  const auto x = casadi::SX::sym("x", model_->nx(), 1);
  const auto z = h(x)[0];
  const auto jac = casadi::SX::jacobian(z, x);
  h_jacs_[name] = casadi::Function("h_jac_" + name, {x}, {jac}, {"x"}, {"H"});

  // update Kalman gain size
  const auto idx_begin = K_.size2();
  const auto idx_end = idx_begin + nz;
  slices_[name] = casadi::Slice(idx_begin, idx_end);
  K_ = casadi::DM::horzcat({K_, casadi::DM::zeros(model_->nx(), nz)});
}

void EKFStateEstimator::compile()
{
  if (K_.size2() == 0) {
    throw NoObservationRegisteredException();
  }
  compiled_ = true;
}

const bool & EKFStateEstimator::is_compiled() const
{
  return compiled_;
}

void EKFStateEstimator::update_observation(
  const StrOpt & name, const casadi::DMDict & in,
  casadi::DMDict & out)
{
  using casadi::DM;
  using casadi::Slice;

  if (!is_compiled()) {
    throw EKFUncompiledException();
  }
  if (name.has_value() && hs_.count(name.value()) == 0) {
    throw ObservationNameNotFoundException(name.value().c_str());
  }

  const auto slice_z = name.has_value() ? slices_[name.value()] : Slice();
  const auto time = static_cast<int64_t>(static_cast<double>(in.at("timestamp")));
  const auto dt = time - nanosec_;

  // TODO(haoru): deal with timestamp jumping back
  if (dt < 0) {
    // for now just output the last estimate if timestamp jumps
    out["x"] = x_;
    out["P"] = P_;
    out["K"] = K_;
    out["Kz"] = K_(Slice(), slice_z);
    return;
  }

  // EKF prediction
  const auto in_dict = casadi::DMDict{{"x", x_}, {"u", u_}, {"dt", dt * 1e-9}};
  const auto x_p = rk4_(in_dict).at("xip1");
  const auto F = F_(in_dict).at("F");
  const auto P_p = DM::mtimes({F, P_, F.T()}) + config_->Q;

  // EKF update
  if (name.has_value()) {
    const auto z = in.at("z");
    const auto R = in.at("R");
    auto h = hs_.at(name.value());
    auto H = h_jacs_.at(name.value())(x_p)[0];
    // innovation
    const auto y = z - h(x_p);
    // innovation covariance
    const auto S = DM::mtimes({H, P_p, H.T() + R});
    // Kalman gain
    K_(Slice(), slice_z) = DM::mtimes({P_p, H.T(), DM::inv(S)});
    // update estimates
    x_ = x_p + DM::mtimes(K_, y);
    // update covariance
    P_ = DM::mtimes(DM::eye(model_->nx()) - DM::mtimes(K_(Slice(), slice_z), H), P_p);
  } else {
    // pure prediction update
    x_ = x_p;
    P_ = P_p;
  }

  // output
  out["x"] = x_;
  out["P"] = P_;
  out["K"] = K_;
  out["Kz"] = K_(Slice(), slice_z);
}

void EKFStateEstimator::update_control(const casadi::DM & u)
{
  u_ = u;
}
}  // namespace ekf_state_estimator
}  // namespace state_estimator
}  // namespace lmpc
