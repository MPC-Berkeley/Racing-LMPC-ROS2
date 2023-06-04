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

#ifndef RACING_LQR__RACING_LQR_CONFIG_HPP_
#define RACING_LQR__RACING_LQR_CONFIG_HPP_

#include <memory>
#include <vector>

#include <casadi/casadi.hpp>

namespace lmpc
{
namespace mpc
{
namespace racing_lqr
{
struct RacingLQRConfig
{
  typedef std::shared_ptr<RacingLQRConfig> SharedPtr;

  size_t N;  // steps
  double dt;  // delta t between steps
  casadi::DM Q;  // state cost-to-go
  casadi::DM R;  // control cost-to-go
  casadi::DM Qf;  // final state cost
};
}  // namespace racing_lqr
}  // namespace mpc
}  // namespace lmpc
#endif  // RACING_LQR__RACING_LQR_CONFIG_HPP_
