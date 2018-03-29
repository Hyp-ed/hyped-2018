/*
 * Author: Martin Kristien
 * Organisation: HYPED
 * Date: 13/03/18
 * Description:
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

#include "sensors/main.hpp"

#include <stdio.h>

#include "data/data.hpp"


namespace hyped {

using data::Data;
using data::Sensors;

namespace sensors {

Main::Main(uint8_t id)
    : Thread(id)
    , data_(data::Data::getInstance())
{ /* EMPTY */ }

void Main::run()
{
  Sensors sensors = {{}, {}, {0, 0}};
  Data& data = Data::getInstance();
  uint32_t time = 0;
  while (1) {
    // keep updating data_ based on values read from sensors
    for (auto& imu : sensors.imu) {
      imu.acc.value[0]  = !imu.acc.value[0];
      imu.acc.timestamp = time;
    }

    data.setSensorsData(sensors);
    sleep(1000);
    time++;
  }
}

}}  // namespace hyped::sensors
