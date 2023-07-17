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

#include <iostream>
#include "lmpc_utils/primitives.hpp"

namespace lmpc
{
std::ostream & operator<<(std::ostream & os, const Position2D & pos)
{
  os << "Position2D(" << pos.x << ", " << pos.y << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const Position3D & pos)
{
  os << "Position3D(" << pos.x << ", " << pos.y << ", " << pos.z << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const BodyVelocity2D & vel)
{
  os << "BodyVelocity2D(" << vel.x << ", " << vel.y << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const SpatialVelocity2D & vel)
{
  os << "SpatialVelocity2D(" << vel.x << ", " << vel.y << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const Pose2D & pose)
{
  os << "Pose2D(" << pose.position.x << ", " << pose.position.y << ", " << pose.yaw << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const FrenetPosition2D & position)
{
  os << "FrenetPosition2D(" << position.s << ", " << position.t << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const FrenetPose2D & pose)
{
  os << "FrenetPose2D(" << pose.position.s << ", " << pose.position.t << ", " << pose.yaw << ")";
  return os;
}

double lateral_sign(const Position2D & position, const Pose2D & p0)
{
  return std::copysign(
    1.0,
    std::cos(p0.yaw) * (position.y - p0.position.y) - std::sin(p0.yaw) *
    (position.x - p0.position.x));
}

double distance(const Position2D & p0, const Position2D & p1)
{
  return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

double distance(const Position3D & p0, const Position3D & p1)
{
  return std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
}

SpatialVelocity2D transform_velocity(const BodyVelocity2D & vb, const double & yaw)
{
  return SpatialVelocity2D{
    vb.x * std::cos(yaw) - vb.y * std::sin(yaw),
    vb.x * std::sin(yaw) + vb.y * std::cos(yaw)};
}

BodyVelocity2D transform_velocity(const SpatialVelocity2D & vs, const double & yaw)
{
  return BodyVelocity2D{
    vs.x * std::cos(yaw) + vs.y * std::sin(yaw),
    -vs.x * std::sin(yaw) + vs.y * std::cos(yaw)};
}
}  // namespace lmpc
