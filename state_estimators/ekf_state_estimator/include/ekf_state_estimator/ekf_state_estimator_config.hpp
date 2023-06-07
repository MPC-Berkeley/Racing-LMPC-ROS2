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

#ifndef EKF_STATE_ESTIMATOR__EKF_STATE_ESTIMATOR_CONFIG_HPP_
#define EKF_STATE_ESTIMATOR__EKF_STATE_ESTIMATOR_CONFIG_HPP_

#include <memory>
#include <vector>

#include <casadi/casadi.hpp>

namespace lmpc
{
namespace state_estimator
{
namespace ekf_state_estimator
{
struct EKFStateEstimatorConfig
{
  typedef std::shared_ptr<EKFStateEstimatorConfig> SharedPtr;

  casadi::DM P0;  // initial estimate covariance
  casadi::DM Q;  // process noise covariance
};
}  // namespace ekf_state_estimator
}  // namespace state_estimator
}  // namespace lmpc
#endif  // EKF_STATE_ESTIMATOR__EKF_STATE_ESTIMATOR_CONFIG_HPP_
