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

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/quaternion.hpp"
#include "utils/math/vector.hpp"

using hyped::data::DataPoint;
using hyped::data::Sensors;
using hyped::utils::math::Quaternion;
using hyped::utils::math::Vector;

namespace hyped {
namespace navigation {

// Public methods
void Navigation::update(const Sensors& data)
{
  // TODO(Uday): Should we check if the data has new data here or in main for navigation?

  gyro_update();
  proximity_orientation_update();
  acclerometer_update();
  stripe_counter_update();
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

DataPoint<Vector<double, 3>> Navigation::proximity_displacement_update()
{
  // TODO(Adi): Calculate displacement from proximity & strip counter. (Point 7)
  return DataPoint<Vector<double, 3>>();
}

void Navigation::stripe_counter_update()
{
  DataPoint<Vector<double, 3>> velocity_1, velocity_2, displacement_1, displacement_2;
  velocity_1 = acc_to_vel.update(accleration_);

  // TODO(Uday): Check if data is new.
  {
    displacement_2 = proximity_displacement_update();
    // TODO(Adi): Find the velocity_2 from displacement_2.

    // TODO(Uday): Set the new velocity using complementary filter.
    displacement_1 = vel_to_dis.update(velocity_);

    // TODO(Uday): Set the new displacement using complementary filter.
  }

  // TODO(Uday): Deal with the case if it is not updated.
}

}}  // namespace hyped::navigation
