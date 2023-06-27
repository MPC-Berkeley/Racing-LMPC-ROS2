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

#ifndef LMPC_UTILS__PRIMITIVES_HPP_
#define LMPC_UTILS__PRIMITIVES_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

namespace lmpc
{
struct Position2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Position3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Velocity2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D
{
  Position2D position;
  double yaw = 0.0;
};

struct FrenetPosition2D
{
  double s = 0.0;
  double t = 0.0;
};

struct FrenetPose2D
{
  FrenetPosition2D position;
  double yaw = 0.0;
};

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

std::ostream & operator<<(std::ostream & os, const Velocity2D & vel)
{
  os << "Velocity2D(" << vel.x << ", " << vel.y << ")";
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


/**
     * @brief use cross product rule to see if a position lies on the left or right of a pose
     *
     * @param position the 2D position to check
     * @param p0 the reference pose
     * @return double 1.0 if the position is on the left, -1.0 if the position is on the right
     */
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
}  // namespace lmpc
#endif  // LMPC_UTILS__PRIMITIVES_HPP_
