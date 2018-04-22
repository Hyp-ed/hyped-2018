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

// #include <stdio.h>

#include "data/data.hpp"


namespace hyped {

using data::Data;
using data::Sensors;

namespace sensors {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log)
    , data_(data::Data::getInstance())
    , bms_(0, log)
{ /* EMPTY */ }

void Main::run()
{
  Sensors sensors = {{}, {}, {0, 0}};
  Data& data = Data::getInstance();
  BMS_Data* bms_data = bms_.getDataPointer();
  uint32_t time = 0;
  while (1) {
    // keep updating data_ based on values read from sensors
    for (auto& imu : sensors.imu) {
      imu.acc.value[0]  = imu.acc.value[0] ? 0 : 2;
      imu.acc.timestamp = time;
    }

    log_.INFO("SENSORS", "BMS voltage %d %d %d %d %d %d %d"
      , bms_data->voltage[0]
      , bms_data->voltage[1]
      , bms_data->voltage[2]
      , bms_data->voltage[3]
      , bms_data->voltage[4]
      , bms_data->voltage[5]
      , bms_data->voltage[6]);
    data.setSensorsData(sensors);
    sleep(1000);
    time++;
  }
}

}}  // namespace hyped::sensors
