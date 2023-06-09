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

#ifndef EKF_STATE_ESTIMATOR__EKF_STATE_ESTIMATOR_HPP_
#define EKF_STATE_ESTIMATOR__EKF_STATE_ESTIMATOR_HPP_

#include <memory>
#include <map>
#include <string>
#include <exception>
#include <optional>

#include <casadi/casadi.hpp>

#include <lmpc_utils/logging.hpp>
#include <single_track_planar_model/single_track_planar_model.hpp>
#include "ekf_state_estimator/ekf_state_estimator_config.hpp"

namespace lmpc
{
namespace state_estimator
{
namespace ekf_state_estimator
{
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModelConfig;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModelConfig;
using lmpc::vehicle_model::single_track_planar_model::XIndex;
using lmpc::vehicle_model::single_track_planar_model::UIndex;

class EKFUninitializedException : public std::exception
{
public:
  const char * what()
  {
    return "Call EKFStateEstimator::initialize() before making any observation updates.";
  }
};

class EKFAlreadyInitializedException : public std::exception
{
public:
  const char * what()
  {
    return "Changes to observations are not allowed after the filter is initialized.";
  }
};

class NoObservationRegisteredException : public std::exception
{
public:
  const char * what()
  {
    return "No observation has been registered for the filter.";
  }
};

class ObservationNameAlreadyExistsException : public std::exception
{
public:
  explicit ObservationNameAlreadyExistsException(const char * name)
  {
    msg_ = "The observation name \"" + std::string(name) + "\" has already been registered.";
  }
  const char * what()
  {
    return msg_.c_str();
  }

protected:
  std::string msg_;
};

class ObservationNameNotFoundException : public std::exception
{
public:
  explicit ObservationNameNotFoundException(const char * name)
  {
    msg_ = "The observation name \"" + std::string(name) + "\" is not found.";
  }
  const char * what()
  {
    return msg_.c_str();
  }

protected:
  std::string msg_;
};

class EKFStateEstimator
{
public:
  typedef std::shared_ptr<EKFStateEstimator> SharedPtr;
  typedef std::unique_ptr<EKFStateEstimator> UniquePtr;
  typedef std::optional<std::string> StrOpt;

  explicit EKFStateEstimator(
    EKFStateEstimatorConfig::SharedPtr ekf_config,
    SingleTrackPlanarModel::SharedPtr model);
  const EKFStateEstimatorConfig & get_config() const;
  const SingleTrackPlanarModel & get_model() const;

  /**
   * @brief Check if the filter has been initialized.
   */
  const bool & is_initialized() const;

  /**
   * @brief Get the latest filter update's timestamp.
   *
   * @return const int64_t& timestamp in nanosecond.
   */
  const int64_t & get_latest_timestamp() const;

  /**
   * @brief Register a new observation for this filter.
   *
   * @param name name for this observation to be referenced during update.
   * @param nz size of this observation.
   * @param h observation function taking nx x 1 state and outputs nz * 1 observation.
   *
   * @throws EKFAlreadyInitializedException if the filter is already initialized.
   * @throws ObservationNameAlreadyExistsException if the observation name is already taken.
   */
  void register_observation(const std::string & name, const casadi_int & nz, casadi::Function & h);

  /**
   * @brief Call this after all observations are registered and before calling any filter updates.
   * No further changes to the observations is allowed afterwards.
   * Re-initialization is also allowed through this function, which will reset state estimate and
   * covariance estimate to initial value in config.
   *
   * @param timestamp nanosecond of time at initialization.
   *
   */
  void initialize(const int64_t & timestamp);

  /**
   * @brief Carry a filter update.
   *
   * @param name observation name at registration. leave empty to carry a pure prediction.
   * @param in observations:
   * - `z` column vector of size nz x 1.
   * - `R` n x n observation covariance matrix.
   * - `timestamp` timestamp of this update in nanosecond.
   *    no update is carried out if this is earlier than the latest time of the filter.
   * @param out output:
   *  - `x` nx x 1 state estimate.
   *  - `P` nx x nx covariance of state estimate.
   *  - `K` Kalman gain of all observations.
   *  - `Kz` Kalman gain with respect to this observation.
   *
   * @throws EKFUninitializedException if calling before the filter is initialized.
   * @throws ObservationNameNotFoundException if the observation name is not registered.
   */
  void update_observation(const StrOpt & name, const casadi::DMDict & in, casadi::DMDict & out);

  /**
   * @brief Updates the control variable.
   *
   * @param u nu x 1 control variable.
   */
  void update_control(const casadi::DM & u);

  /**
   * @brief Get access to the EKF logger to listen to callbacks.
   *
   * @return utils::Logger& internal logger object.
   */
  utils::Logger & get_logger();

protected:
  typedef std::map<std::string, casadi::Function> FunctionDict;
  typedef std::map<std::string, casadi::Slice> SliceDict;

  EKFStateEstimatorConfig::SharedPtr config_ {};
  SingleTrackPlanarModel::SharedPtr model_ {};
  casadi::Function rk4_;
  casadi::Function F_;  // jacobian of discrete dynamics w.r.t. state

  bool initialized_;  // signal if all observations are registered.
  FunctionDict hs_;  // store (h) observation models
  FunctionDict h_jacs_;  // store (H) jacobian of observation models w.r.t. state
  SliceDict slices_;  // store how to slice the Kalman gain for a certain observation

  casadi::DM x_;  // state estimate
  casadi::DM u_;  // control variable
  casadi::DM P_;  // estimate covariance
  casadi::DM K_;  // Kalman gain
  int64_t nanosec_;  // timestamp of the last update

  utils::Logger logger_;

  /**
   * @brief invokes m.is_regular()
   * and sends a log to dump the matrix if NaN or infinity exists.
   *
   * @param m matrix
   * @return true (ok) if no NaN or infinity exists.
   * @return false (bad) if NaN or infinity exist.
   */
  bool check_nan_inf(const casadi::DM & m, const std::string & name);

  /**
   * @brief sanity check the covariance matrix.
   * all elements must be non-negative. diagnals must be positive.
   * negative elements are zeroed out.
   * non-positive diagnal elements are set to 1e-6.
   *
   * @param cov
   * @return true if the covariance is ok and no modification was done.
   * @return false some modification was done as described above.
   */
  bool check_cov(casadi::DM & cov, const std::string & name);
};
}  // namespace ekf_state_estimator
}  // namespace state_estimator
}  // namespace lmpc
#endif  // EKF_STATE_ESTIMATOR__EKF_STATE_ESTIMATOR_HPP_
