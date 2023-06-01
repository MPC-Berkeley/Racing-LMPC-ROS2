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

#ifndef BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_HPP_
#define BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_HPP_

#include <memory>

#include <casadi/casadi.hpp>

#include "base_vehicle_model/base_vehicle_model_config.hpp"

namespace lmpc
{
namespace vehicle_model
{
namespace base_vehicle_model
{

struct BaseVehicleModelState
{
  double wheel_speed_fl_rps = 0.0;
  double wheel_speed_fr_rps = 0.0;
  double wheel_speed_rl_rps = 0.0;
  double wheel_speed_rr_rps = 0.0;
  double engine_rpm = 0.0;
  uint8_t gear = 1;
};

class BaseVehicleModel
{
public:
  typedef std::shared_ptr<BaseVehicleModel> SharedPtr;
  typedef std::unique_ptr<BaseVehicleModel> UniquePtr;

  explicit BaseVehicleModel(BaseVehicleModelConfig::SharedPtr config);

  void set_base_config(BaseVehicleModelConfig::SharedPtr config);
  const BaseVehicleModelConfig & get_base_config() const;

  BaseVehicleModelState & get_state();
  const BaseVehicleModelState & get_const_state() const;

  /**
   * @brief Get the size of the state variable.
   *
   * @return state variable size.
   */
  virtual size_t nx() const;

  /**
   * @brief Get the size of the control variable.
   *
   * @return control variable size.
   */
  virtual size_t nu() const;

  /**
   * @brief Override to implement dynamics.
   *
   * @param in "x" (state) and "u" (control) are required. additional inputs are optional.
   * @param out "x_dot" (time derivative of state) is required. additional outputs are optional.
   */
  virtual void forward_dynamics(const casadi::DMDict & in, casadi::DMDict & out);

/**
 * @brief Override to implement Jacobian of dynamics
 *
 * @param in "x" (state) and "u" (control)
 * @param out Jacobian of "x_dot" with respect to x (A matrix) and u (B matrix) evaulated at their given point.
 */
  virtual void dynamics_jacobian(const casadi::DMDict & in, casadi::DMDict & out);

  /**
   * @brief Add constraints to the optimal control problem.
   *
   * @param opti Casadi NLP optimizer
   * @param in "x" (state), "u" (control), "t"(delta t), "xip1" (next state), "uip1" (next control).
   */
  virtual void add_nlp_constraints(casadi::Opti & opti, const casadi::MXDict & in);

  /**
   * @brief calculate longitudinal control based on control variable.
   *
   * @param in "u" (control) required, the rest is optional.
   * @param throttle output throttle in 1-100.
   * @param brake_kpa output brake in kpa.
   */
  virtual void calc_lon_control(const casadi::DMDict & in, double & throttle, double & brake_kpa);

  /**
   * @brief calculate lateral control based on control variable.
   *
   * @param in "u" (control) required, the rest is optional.
   * @param steering_rad output front wheel angle in radian.
   */
  virtual void calc_lat_control(const casadi::DMDict & in, double & steering_rad);

protected:
  BaseVehicleModelConfig::SharedPtr base_config_ {};
  BaseVehicleModelState base_state_;

  /**
   * @brief calculate throttle based on drive force.
   *        torque vs. RPM needs to monotonically and linearly decrease.
   *        not designed for gas engine car.
   *        override to customize.
   *        `base_state_` needs to be populated beforehand.
   *
   * @param fd total driving force.
   * @return target throttle 1-100.
   */
  virtual double calc_throttle(const double & fd);

  /**
   * @brief calculate brake line pressure based on braking force.
   *        pressure is taken at the master cylinder.
   *        this sums front and rear pressures.
   *        override to customize.
   *        `base_state_` needs to be populated beforehand.
   *
   * @param fb total braking force.
   * @return target brake line pressure in kpa
   */
  virtual double calc_brake(const double & fb);
};
}  // namespace base_vehicle_model
}  // namespace vehicle_model
}  // namespace lmpc
#endif  // BASE_VEHICLE_MODEL__BASE_VEHICLE_MODEL_HPP_
