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

#include "data/data.hpp"

namespace hyped {

using data::Data;
using data::Sensors;
using data::Batteries;

namespace sensors {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      imu_manager_(id, log),
      proxi_manager_front_(id, log, true),
      proxi_manager_back_(id, log, false),
      battery_manager_lp(id, log)
{
  // Config new IMU manager
  imu_manager_.config(&sensors_.imu);

  // Config Proxi manager
  proxi_manager_front_.config(&sensors_.proxi_front);
  proxi_manager_back_.config(&sensors_.proxi_back);

  // Config BMS Manager
  battery_manager_lp.config(&batteries_.low_power_batteries);
}

void Main::run()
{
  while (1) {
    // Write to the data structure here

    // Update sensor data structure
    data_.setSensorsData(sensors_);
    yield();
  }
}
}}  // namespace hyped::sensors
