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

#include <string>
#include <memory>
#include <vector>

#include <lmpc_utils/ros_param_helper.hpp>

#include "ekf_state_estimator/ros_param_loader.hpp"

namespace lmpc
{
namespace state_estimator
{
namespace ekf_state_estimator
{
EKFStateEstimatorConfig::SharedPtr load_parameters(rclcpp::Node * node)
{
  auto declare_vec = [&](const char * name) {
      return lmpc::utils::declare_parameter<std::vector<double>>(node, name);
    };
  return std::make_shared<EKFStateEstimatorConfig>(
    EKFStateEstimatorConfig{
          casadi::DM::reshape(casadi::DM(declare_vec("ekf_state_estimator.p0")), 6, 6),
          casadi::DM::reshape(casadi::DM(declare_vec("ekf_state_estimator.q")), 6, 6),
        }
  );
}
}  // namespace ekf_state_estimator
}  // namespace state_estimator
}  // namespace lmpc
