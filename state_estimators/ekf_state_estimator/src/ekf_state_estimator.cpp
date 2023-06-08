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
  initialized_(false), hs_(), h_jacs_(), x_(config_->x0), u_(model_->nu(), 1),
  P_(config_->P0), K_(model_->nx(), 0)
{
  // build jacobian of discrete dynamics
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

const bool & EKFStateEstimator::is_initialized() const
{
  return initialized_;
}

void EKFStateEstimator::register_observation(
  const std::string & name, const casadi_int & nz,
  casadi::Function & h)
{
  if (is_initialized()) {
    throw EKFAlreadyInitializedException();
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

void EKFStateEstimator::initialize(const int64_t & timestamp)
{
  if (K_.size2() == 0) {
    throw NoObservationRegisteredException();
  }
  initialized_ = true;
  nanosec_ = timestamp;
  x_ = config_->x0;
  P_ = config_->P0;
}

void EKFStateEstimator::update_observation(
  const StrOpt & name, const casadi::DMDict & in,
  casadi::DMDict & out)
{
  using casadi::DM;
  using casadi::Slice;

  if (!is_initialized()) {
    throw EKFUninitializedException();
  }
  if (name.has_value() && hs_.count(name.value()) == 0) {
    throw ObservationNameNotFoundException(name.value().c_str());
  }

  const auto slice_z = name.has_value() ? slices_[name.value()] : Slice();
  const auto time = static_cast<int64_t>(static_cast<double>(in.at("timestamp")));
  const auto dt = time - nanosec_;

  // timestamp jumps back? reset the fitler.
  if (dt < 0) {
    initialize(time);
  }

  // EKF prediction
  const auto in_dict = casadi::DMDict{{"x", x_}, {"u", u_}, {"dt", dt * 1e-9}};
  const auto & x_p = rk4_(in_dict).at("xip1");
  const auto F = F_(in_dict).at("F");
  const auto P_p = DM::mtimes({F, P_, F.T()}) + config_->Q;

  // EKF update
  if (name.has_value()) {
    const auto & z = in.at("z");
    auto R = in.at("R");
    if (!(check_nan_inf(
        z,
        "input observation z") && check_nan_inf(R, "input observation covariance R")))
    {
      // NaN and Inf check fails for this input. Carry out pure prediction.
      logger_.send_log(
        utils::LogLevel::WARN,
        "NaN or Inf detected in filter input. Falling back to a pure prediction update.");
      x_ = x_p;
      P_ = P_p;
    } else {
      check_cov(R, "input observation covariance R");
      // carry out normal EKF update.
      auto & h = hs_.at(name.value());
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
    }
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

utils::Logger & EKFStateEstimator::get_logger()
{
  return logger_;
}

bool EKFStateEstimator::check_nan_inf(const casadi::DM & m, const std::string & name)
{
  if (m.is_regular()) {
    return false;
  } else {
    const auto what = "NaN or Inf detected in matrix \"" + name + "\":\n" + m.get_str();
    logger_.send_log(utils::LogLevel::ERROR, what);
    return true;
  }
  return false;
}

bool EKFStateEstimator::check_cov(casadi::DM & cov, const std::string & name)
{
  using casadi::DM;
  bool has_issue = false;
  const auto zero = DM::zeros(1, 1);

  for (casadi_int i = 0; i < cov.size1(); i++) {
    for (casadi_int j = 0; i < cov.size2(); i++) {
      if (static_cast<casadi_int>(cov(i, j) < 0.0)) {
        has_issue = true;
        cov(i, j) = 0.0;
      }
      if (i == j && static_cast<casadi_int>(cov(i, j) <= 0.0)) {
        has_issue = true;
        cov(i, j) = 1e-6;
      }
    }
  }

  if (has_issue) {
    const auto what = "Covariance matrix \"" + name +
      "\" is ill-formed. It must be non-negative and have positive diagnals:\n" +
      cov.get_str();
    logger_.send_log(utils::LogLevel::WARN, what);
  }
  return !has_issue;
}
}  // namespace ekf_state_estimator
}  // namespace state_estimator
}  // namespace lmpc
