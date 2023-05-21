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

#ifndef LMPC_UTILS__LOOKUP_HPP_
#define LMPC_UTILS__LOOKUP_HPP_

#include <memory>
#include <string>
#include <vector>

namespace lmpc
{
namespace utils
{
struct Lookup2D
{
  std::vector<double> x;
  std::vector<double> y;
};

struct Lookup3D
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

double fast_linear_interpolate(
  const double & x_min, const double & x_max, const double & y_min, const double & y_max,
  const double & x_val, const bool & extrapolate);
double linear_interpolate(const Lookup2D & lookup, const double & x, const bool & extrapolate);
double bilinear_interpolate(
  const Lookup3D & lookup, const double & x, const double & y,
  const bool & extrapolate);
}  // namespace utils
}  // namespace lmpc
#endif  // LMPC_UTILS__LOOKUP_HPP_
