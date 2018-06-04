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
    : Thread(id, log)
    , data_(data::Data::getInstance())
{
  // create BMS LP
  for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
    bms_[i] = new BMS(i, &batteries_.low_power_batteries[i] , log_);
    bms_[i]->start();
  }

  // create Proximities
  for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    proxi_[i] = new VL6180(0x29, log_);
    proxi_[i]->setContinuousRangingMode();
  }
}

void Main::run()
{
  while (1) {
    // keep updating data_ based on values read from sensors
    for (BMS* bms: bms_) {
      bms->update();
    }

    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      sensors_.proxi[i].val = proxi_[i]->getDistance();
    }

    data_.setSensorsData(sensors_);
    data_.setBatteryData(batteries_);
    sleep(1000);
  }
}

}}  // namespace hyped::sensors
