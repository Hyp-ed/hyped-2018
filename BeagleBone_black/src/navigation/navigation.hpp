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
#include "utils/math/kalman.hpp"
#include "utils/math/vector.hpp"

using hyped::utils::math::Kalman;
using hyped::utils::math::Vector;

namespace hyped {
namespace navigation {

class Navigation {
 public:
  // TODO(Uday,Brano,Adi): Change the return values for the private methods to vector/quaternion

  void update();
  float get_accleration();
  float get_velocity();
  float get_displacement();

 private:
  // TODO(Uday,Brano,Adi): Add the arguements for the private methods

  void gyro_update();  // Point number 1
  void acclerometer_update();  // Point number 3, 4, 5 and 6
  void proximity_orientation_update();  // Point number 7
  void proximity_displacement_update();  // Point number 7
  void stripe_counter_update();  // Point number 7
  // float velocity_update();  // Point number 4 and 5

  // TODO(Uday,Brano,Adi): Change the data types of the data members

  Vector<float, 3> accleration_;
  Vector<float, 3> velocity_;
  Vector<float, 3> displacement_;
  Vector<float, 3> orientation_;
  int stripe_count_;
  Kalman<Vector<int16_t, 3>> acclerometer_filter_;
  Kalman<Vector<int16_t, 3>> gyro_filter_;
  // TODO(Uday,Brano,Adi): Decide the type
  Kalman<uint8_t> proximity_filter_;
};

}}  // namespace hyped::navigation

#endif  // BEAGLEBONE_BLACK_NAVIGATION_MAIN_HPP_
