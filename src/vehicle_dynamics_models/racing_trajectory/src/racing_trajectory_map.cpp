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

#include <regex>
#include <string>
#include <filesystem>

#include <racing_trajectory/racing_trajectory_map.hpp>

namespace lmpc
{
namespace vehicle_model
{
namespace racing_trajectory
{

RacingTrajectoryMap::RacingTrajectoryMap(const std::string & directory_path)
{
  std::regex pattern(R"((\d+)_.*\.txt)");

  for (const auto & entry : std::filesystem::directory_iterator(directory_path)) {
    std::smatch match;
    std::string file_name = entry.path().filename().string();

    if (!std::regex_match(file_name, match, pattern)) {
      throw std::runtime_error(
              "File does not conform to the required format [number_name.txt]: " + file_name);
    }

    int number = std::stoi(match[1]);
    if (trajectories_.find(number) != trajectories_.end()) {
      throw std::runtime_error("Duplicate trajectory number found: " + std::to_string(number));
    }
    trajectories_[number] = std::make_shared<RacingTrajectory>(entry.path().string());
  }
}

RacingTrajectory::SharedPtr RacingTrajectoryMap::get_trajectory(const int & index)
{
  if (trajectories_.find(index) == trajectories_.end()) {
    std::cerr << "Trajectory number " << index << " not found." << std::endl;
    return nullptr;
  }
  return trajectories_[index];
}
}  // namespace racing_trajectory
}  // namespace vehicle_model
}  // namespace lmpc
