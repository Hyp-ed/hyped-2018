/*
 * Author: Branislav Pilnan
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description: Class for data exchange between sub-team threads and structures for holding data
 *              produced by each of the sub-teams.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "data/data.hpp"

#include <stdio.h>

using hyped::data::Data;
using hyped::data::Navigation;
using hyped::data::Sensors;

int main()
{
  Data& data = Data::getInstance();

  Navigation  nav   = data.getNavigationData();
  Sensors     sens  = data.getSensorsData();

  printf("Hello hyped, here are some shared data\n");
  printf("from Navigation: %f %f %f\n", nav.distance, nav.velocity, nav.acceleration);
  printf("from Sensors: %f %d %d at time %d\n",
      sens.imu.value[0].acc[0],
      sens.proxi_front.value[0].val,
      sens.keyence_stripe_counter[0].count.value,
      sens.keyence_stripe_counter[0].count.timestamp);
  return 0;
}
