/*
 * Author: Adithya Sireesh, Uday Patel
 * Organisation: HYPED
 * Date: 10/02/2018
 * Description: Navigation class functionality definition.
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

#include "navigation.hpp"

#include "data/data_point.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/quaternion.hpp"
#include "utils/math/vector.hpp"

using hyped::data::DataPoint;
using hyped::utils::math::Quaternion;
using hyped::utils::math::Vector;

namespace hyped {
namespace navigation {

// Public methods
void Navigation::update()
{
  gyro_update();
  proximity_orientation_update();
  acclerometer_update();

  DataPoint<Vector<double, 3>> velocity_1 = acc_to_vel.update(accleration_);  // Point 4

  proximity_displacement_update();
  DataPoint<Vector<double, 3>> velocity_2;
  // TODO(Adi): Find the velocity from displacement_1. (Save to velocity_2)
  // TODO(Uday): Set the new velocity using complementary filter.

  DataPoint<Vector<double, 3>> displacement_1 = vel_to_dis.update(velocity_);  // Point 6
  // TODO(Uday): Set the new displacement using complementary filter.
}

Vector<double, 3> Navigation::get_accleration()
{
  return accleration_.value;
}

Vector<double, 3> Navigation::get_velocity()
{
  return velocity_.value;
}

Vector<double, 3> Navigation::get_displacement()
{
  return displacement_.value;
}

// Private methods
void Navigation::gyro_update()
{
  // TODO(Adi): Calculate Point 1 of the FDP.
}

void Navigation::acclerometer_update()
{
  // TODO(Adi): Calculate Point 3 of the FDP.
}

void Navigation::proximity_orientation_update()
{
  // TODO(Adi): Calculate SLERP (Point 2 of the FDP).
}

void Navigation::proximity_displacement_update()
{
  // TODO(Adi): Calculate displacement from proximity & strip counter. (Point 7)
  // NOTE(Adi): This method will call stripe_counter_update().
}

void Navigation::stripe_counter_update()
{
  // TODO(Uday): Get data and return it.
}

}}  // namespace hyped::navigation
