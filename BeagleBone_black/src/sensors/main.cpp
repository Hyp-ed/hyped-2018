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
#include "sensors/can_proxi.hpp"
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
      proxi_manager_(id, log)
{
  // create BMS LP
  for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
    BMS* bms = new BMS(i, &batteries_.low_power_batteries[i], log_);
    bms->start();
    bms_[i] = bms;
  }

  // // TODO(anyone): change this to use CAN-based proxies
  // for (int i = 0; i < data::Sensors::kNumProximities; i++) {
  //   // initialisation of all proxi sensors
  //   VL6180* proxi = new VL6180(0x29, log_);
  //   proxi->setContinuousRangingMode();
  //   can_proxi_[i] = proxi;
  // }


  // Create Proxi manager
  proxi_manager_.config(&sensors_.proxi);

  // create CAN-based proximities
  for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    CanProxi* proxi = new CanProxi(i, log_);
    can_proxi_[i] = proxi;
  }


  // Config new IMU manager
  imu_manager_.config(&sensors_.imu);
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

// void Main::updateProxi()
// {
//   // update front cluster of proximities
//   for (int i = 0; i < data::Sensors::kNumProximities; i++) {
//     proxi_[i]->getData(&sensors_.proxi_front[i]);
//   }
//
//   // update back cluster of proximities
//   for (int i = 0; i < data::Sensors::kNumProximities; i++) {
//     can_proxi_[i]->getData(&sensors_.proxi_back[i]);
//   }
//   sleep(10);
// }

}}  // namespace hyped::sensors
