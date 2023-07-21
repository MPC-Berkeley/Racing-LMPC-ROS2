// Copyright 2022 AI Racing Tech
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

#ifndef TRANSFORM_HELPER__TRANSFORM_HELPER_HPP_
#define TRANSFORM_HELPER__TRANSFORM_HELPER_HPP_

#include <memory>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <lmpc_utils/primitives.hpp>

namespace lmpc
{
namespace utils
{
class TransformHelper
{
public:
  explicit TransformHelper(rclcpp::Node & node);

  bool lookup_transform(
    const std::string & source_frame, const std::string & target_frame,
    geometry_msgs::msg::TransformStamped & transform, const rclcpp::Time & time);

  void send_transform(const geometry_msgs::msg::TransformStamped & msg);

  template<typename T>
  static T calc_yaw_difference(const T & yaw_1, const T & yaw_2);

  static double heading_from_quaternion(const geometry_msgs::msg::Quaternion q);
  static double heading_from_quaternion(const tf2::Quaternion q);
  static tf2::Quaternion quaternion_from_heading(const double & yaw);
  static void do_transform(
    const Position2D & in, Position2D & out,
    const geometry_msgs::msg::TransformStamped & transform);
  static void do_transform(
    const Position3D & in, Position3D & out,
    const geometry_msgs::msg::TransformStamped & transform);

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node & node_;
};
}  // namespace utils
}  // namespace lmpc
#endif  // TRANSFORM_HELPER__TRANSFORM_HELPER_HPP_
