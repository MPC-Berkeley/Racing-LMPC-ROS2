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

#ifndef RACING_SIMULATOR__RACING_SIMULATOR_HPP_
#define RACING_SIMULATOR__RACING_SIMULATOR_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "racing_simulator/racing_simulator_config.hpp"
#include <single_track_planar_model/single_track_planar_model.hpp>
#include <racing_trajectory/racing_trajectory.hpp>

namespace lmpc
{
namespace simulation
{
namespace racing_simulator
{
using lmpc::vehicle_model::base_vehicle_model::BaseVehicleModelConfig;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModelConfig;
using lmpc::vehicle_model::single_track_planar_model::XIndex;
using lmpc::vehicle_model::single_track_planar_model::UIndex;
using lmpc::vehicle_model::racing_trajectory::RacingTrajectory;

class RacingSimulator
{
public:
  typedef std::shared_ptr<RacingSimulator> SharedPtr;
  typedef std::unique_ptr<RacingSimulator> UniquePtr;

  explicit RacingSimulator(
    const double & dt,
    const casadi::DM & x0,
    RacingTrajectory::SharedPtr track,
    SingleTrackPlanarModel::SharedPtr model);

  SingleTrackPlanarModel & get_model();
  RacingTrajectory & get_track();

  const casadi::DM & x() const;
  const casadi::DM & u() const;

  void set_state(const casadi::DM & x);
  void step(const casadi::DM & u);

protected:
  casadi::DM dt_;
  casadi::DM x_;
  casadi::DM u_;

  RacingTrajectory::SharedPtr track_ {};
  SingleTrackPlanarModel::SharedPtr model_ {};

  casadi::Function discrete_dynamics_ {};
};
}  // namespace racing_simulator
}  // namespace simulation
}  // namespace lmpc
#endif  // RACING_SIMULATOR__RACING_SIMULATOR_HPP_
