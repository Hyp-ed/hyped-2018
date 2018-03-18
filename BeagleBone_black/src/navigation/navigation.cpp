/*
 * Author: Adithya Sireesh
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


namespace hyped {
namespace navigation {

// Public methods
void Navigation::update() {}

float Navigation::get_accleration()
{
  return 0;
}

float Navigation::get_velocity()
{
  return 0;
}

float Navigation::get_displacement()
{
  return 0;
}

// Private methods

void Navigation::gyro_update() {}

void Navigation::acclerometer_update() {}

void Navigation::proximity_orientation_update() {}

void Navigation::proximity_displacement_update() {}

void Navigation::stripe_counter_update() {}

}}  // namespace hyped::navigation
