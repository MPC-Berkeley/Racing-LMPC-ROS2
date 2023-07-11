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

#include <vector>
#include "lmpc_utils/lookup.hpp"

namespace lmpc
{
namespace utils
{
size_t find_index(const std::vector<double> & list, const double & val)
{
  const size_t size = list.size();
  size_t i = 0;
  if (val >= list[size - 2]) {
    i = size - 2;
  } else {
    while (val > list[i + 1]) {i++;}
  }
  return i;
}

double fast_linear_interpolate(
  const double & x_min, const double & x_max, const double & y_min, const double & y_max,
  const double & x_val, const bool & extrapolate)
{
  double yL = y_min, yR = y_max;
  if (!extrapolate) {
    if (x_val < x_min) {yR = yL;}
    if (x_val > x_max) {yL = yR;}
  }
  const double dydx = ( yR - yL ) / ( x_max - x_min );
  return yL + dydx * ( x_val - x_min );
}

double linear_interpolate(const Lookup2D & lookup, const double & x, const bool & extrapolate)
{
  const auto idx = find_index(lookup.x, x);
  return fast_linear_interpolate(
    lookup.x[idx], lookup.x[idx + 1], lookup.y[idx], lookup.y[idx + 1],
    x, extrapolate);
}

double bilinear_interpolate(
  const Lookup3D & lookup, const double & x, const double & y,
  const bool & extrapolate)
{
  const auto xL = find_index(lookup.x, x);
  const auto xR = xL + 1;
  const auto yL = find_index(lookup.y, y);
  const auto yR = yL + 1;

  const auto val_1 = fast_linear_interpolate(
    lookup.y[yL], lookup.y[yR],
    lookup.z[xL * lookup.y.size() + yL], lookup.z[xL * lookup.y.size() + yR],
    y, extrapolate);
  const auto val_2 = fast_linear_interpolate(
    lookup.y[yL], lookup.y[yR],
    lookup.z[xR * lookup.y.size() + yL], lookup.z[xR * lookup.y.size() + yR],
    y, extrapolate);

  return fast_linear_interpolate(lookup.x[xL], lookup.x[xR], val_1, val_2, x, extrapolate);
}
}  // namespace utils
}  // namespace lmpc
