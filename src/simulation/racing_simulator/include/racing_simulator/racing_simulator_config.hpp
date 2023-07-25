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

#ifndef RACING_SIMULATOR__RACING_SIMULATOR_CONFIG_HPP_
#define RACING_SIMULATOR__RACING_SIMULATOR_CONFIG_HPP_

#include <memory>
#include <string>

#include <casadi/casadi.hpp>

namespace lmpc
{
namespace simulation
{
namespace racing_simulator
{
enum RacingSimulatorStepMode
{
  STEP,
  CONTINUOUS
};

struct RacingSimulatorConfig
{
  typedef std::shared_ptr<RacingSimulatorConfig> SharedPtr;
  double dt = 0.0;
  double repeat_state_dt = 0.0;
  bool publish_tf = false;
  bool visualize_boundary = true;
  bool visualize_abscissa = true;
  bool visualize_vehicle = true;
  std::string race_track_file_path = "";
  RacingSimulatorStepMode step_mode = RacingSimulatorStepMode::STEP;
  casadi::DM x0;
};
}  // namespace racing_simulator
}  // namespace simulation
}  // namespace lmpc
#endif  // RACING_SIMULATOR__RACING_SIMULATOR_CONFIG_HPP_
