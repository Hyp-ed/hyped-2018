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

#include "sensors/bms.hpp"
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
      proxi_manager_back_(id, log, false)
{
  // // create BMS LP
  // for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
  //   BMS* bms = new BMS(i, &batteries_.low_power_batteries[i], log_);
  //   bms->start();
  //   bms_[i] = bms;
  // }
  // Config new IMU manager
  imu_manager_.config(&sensors_.imu);

  // Create Proxi manager
  proxi_manager_front_.config(&sensors_.proxi_front);
  proxi_manager_back_.config(&sensors_.proxi_back);
}

void Main::run()
{
  // Create 2 seperate threads for imu and proxi data
  // Get the data maybe a imu 1:3 proxi ratio
  while (1) {
    // Write to the data structure here

    // Update sensor data structure
    data_.setSensorsData(sensors_);
    yield();
  }
}

// void Main::updateBms()
// {
//   // keep updating data_ based on values read from sensors
//   // update BMS LP
//   for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
//     bms_[i]->getData(&batteries_.low_power_batteries[i]);
//   }
//
//   // Update battery data structure
//   batteries_ = data_.getBatteriesData();
//   bms_[0]->getData(&batteries.low_power_batteries[0]);
//   data_.setBatteryData(batteries_);
//
//   sleep(100);
// }
}}  // namespace hyped::sensors
