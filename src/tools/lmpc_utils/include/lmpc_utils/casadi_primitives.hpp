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

#ifndef LMPC_UTILS__CASADI_PRIMITIVES_HPP_
#define LMPC_UTILS__CASADI_PRIMITIVES_HPP_

#include <iostream>
#include <casadi/casadi.hpp>

namespace lmpc
{
template<typename T = casadi::DM>
struct CasadiPosition2D
{
  T x = 0.0;
  T y = 0.0;
};

template<typename T = casadi::DM>
struct CasadiPosition3D
{
  T x = 0.0;
  T y = 0.0;
  T z = 0.0;
};

template<typename T = casadi::DM>
struct CasadiBodyVelocity2D
{
  T x = 0.0;
  T y = 0.0;
  T v_yaw = 0.0;
};

template<typename T = casadi::DM>
struct CasadiSpatialVelocity2D
{
  T x = 0.0;
  T y = 0.0;
  T v_yaw = 0.0;
};

template<typename T = casadi::DM>
struct CasadiPose2D
{
  CasadiPosition2D<T> position;
  T yaw = 0.0;
};

template<typename T = casadi::DM>
struct CasadiFrenetPosition2D
{
  T s = 0.0;
  T t = 0.0;
};

template<typename T = casadi::DM>
struct CasadiFrenetPose2D
{
  CasadiFrenetPosition2D<T> position;
  T yaw = 0.0;
};

// std::ostream & operator<<(std::ostream & os, const Position2D & pos);
// std::ostream & operator<<(std::ostream & os, const Position3D & pos);
// std::ostream & operator<<(std::ostream & os, const BodyVelocity2D & vel);
// std::ostream & operator<<(std::ostream & os, const SpatialVelocity2D & vel);
// std::ostream & operator<<(std::ostream & os, const Pose2D & pose);
// std::ostream & operator<<(std::ostream & os, const FrenetPosition2D & position);
// std::ostream & operator<<(std::ostream & os, const FrenetPose2D & pose);

template<typename T = casadi::DM>
double distance(const CasadiPosition2D<T> & p0, const CasadiPosition2D<T> & p1)
{
  return T::hypot(p1.x - p0.x, p1.y - p0.y);
}

template<typename T = casadi::DM>
double distance(const CasadiPosition3D<T> & p0, const CasadiPosition3D<T> & p1)
{
  return T::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
}

template<typename T = casadi::DM>
CasadiSpatialVelocity2D<T> transform_velocity(const CasadiBodyVelocity2D<T> & vb, const T & yaw)
{
  return CasadiSpatialVelocity2D<T>{
    vb.x * cos(yaw) - vb.y * sin(yaw),
    vb.x * sin(yaw) + vb.y * cos(yaw),
    vb.v_yaw
  };
}

template<typename T = casadi::DM>
CasadiBodyVelocity2D<T> transform_velocity(const CasadiSpatialVelocity2D<T> & vs, const T & yaw)
{
  return CasadiBodyVelocity2D<T>{
    vs.x * std::cos(yaw) + vs.y * std::sin(yaw),
    -vs.x * std::sin(yaw) + vs.y * std::cos(yaw),
    vs.v_yaw
  };
}

}  // namespace lmpc
#endif  // LMPC_UTILS__CASADI_PRIMITIVES_HPP_
