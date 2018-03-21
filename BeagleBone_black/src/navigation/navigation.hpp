/*
 * Author: Adithya Sireesh
 * Organisation: HYPED
 * Date: 10/02/2018
 * Description: Navigation class skeleton.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_
#define BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_

#include <cstdint>

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "utils/math/kalman.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/quaternion.hpp"
#include "utils/math/vector.hpp"

using hyped::data::DataPoint;
using hyped::data::Sensors;
using hyped::utils::math::Kalman;
using hyped::utils::math::Integrator;
using hyped::utils::math::Quaternion;
using hyped::utils::math::Vector;

namespace hyped {
namespace navigation {

class Navigation {
 public:
  // TODO(ALL): Change the return values for the private methods to vector/quaternion

  void update(const Sensors& data);
  Vector<double, 3> get_accleration();
  Vector<double, 3> get_velocity();
  Vector<double, 3> get_displacement();

 private:
  // TODO(ALL): Add the arguements for the private methods

  void gyro_update();  // Point number 1
  void acclerometer_update();  // Point number 3, 4, 5 and 6
  void proximity_orientation_update();  // Point number 7
  void proximity_displacement_update();  // Point number 7
  void stripe_counter_update();  // Point number 7
  // double velocity_update();  // Point number 4 and 5

  // TODO(ALL): Change the data types of the data members
  DataPoint<Vector<double, 3>> accleration_;
  DataPoint<Vector<double, 3>> velocity_;
  DataPoint<Vector<double, 3>> displacement_;
  DataPoint<Vector<double, 3>> orientation_;
  int stripe_count_;
  Kalman<Vector<int16_t, 3>> acclerometer_filter_;
  Kalman<Vector<int16_t, 3>> gyro_filter_;
  // TODO(ALL): Decide the type
  Kalman<uint8_t> proximity_filter_;

  Integrator<Vector<double, 3>> acc_to_vel;
  Integrator<Vector<double, 3>> vel_to_dis;
};

}}  // namespace hyped::navigation

#endif  // BEAGLEBONE_BLACK_NAVIGATION_MAIN_HPP_
