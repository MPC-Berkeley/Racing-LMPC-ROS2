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

#ifndef DOUBLE_TRACK_PLANAR_MODEL__ROS_PARAM_LOADER_HPP_
#define DOUBLE_TRACK_PLANAR_MODEL__ROS_PARAM_LOADER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "double_track_planar_model/double_track_planar_model.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace double_track_planar_model
{
DoubleTrackPlanarModelConfig::SharedPtr load_parameters(rclcpp::Node * node);
}  // namespace double_track_planar_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // DOUBLE_TRACK_PLANAR_MODEL__ROS_PARAM_LOADER_HPP_
