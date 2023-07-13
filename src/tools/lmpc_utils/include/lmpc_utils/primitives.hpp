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
#include <casadi/casadi.hpp>

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

std::ostream & operator<<(std::ostream & os, const Position2D & pos);
std::ostream & operator<<(std::ostream & os, const Position3D & pos);
std::ostream & operator<<(std::ostream & os, const Velocity2D & vel);
std::ostream & operator<<(std::ostream & os, const Pose2D & pose);
std::ostream & operator<<(std::ostream & os, const FrenetPosition2D & position);
std::ostream & operator<<(std::ostream & os, const FrenetPose2D & pose);


/**
     * @brief use cross product rule to see if a position lies on the left or right of a pose
     *
     * @param position the 2D position to check
     * @param p0 the reference pose
     * @return double 1.0 if the position is on the left, -1.0 if the position is on the right
     */
double lateral_sign(const Position2D & position, const Pose2D & p0);

template<typename T>
T lateral_sign(const T & position, const T & p0)
{
  using casadi::Slice;
  return sign(
    cos(p0(2, Slice())) * (position(1, Slice()) - p0(1, Slice())) -
    sin(p0(2, Slice())) * (position(0, Slice()) - p0(0, Slice()))
  );
}

double distance(const Position2D & p0, const Position2D & p1);
double distance(const Position3D & p0, const Position3D & p1);
}  // namespace lmpc
#endif  // LMPC_UTILS__PRIMITIVES_HPP_
