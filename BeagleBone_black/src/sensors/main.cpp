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
#include "sensors/mpu9250.hpp"
#include "sensors/vl6180.hpp"
#include "data/data.hpp"

namespace hyped {

using data::Data;
using data::Sensors;
using data::Batteries;

namespace sensors {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      chip_select_ {31, 50, 48, 51}
{
  // create BMS LP
  for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
    BMS* bms = new BMS(i, log_);
    bms->start();
    bms_[i] = bms;
  }

  // create Proximities
  for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    VL6180* proxi = new VL6180(0x29, log_);
    proxi->setContinuousRangingMode();
    proxi_[i] = proxi;
  }

  // create CAN-based proximities
  for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    CanProxi* proxi = new CanProxi(i, log_);
    can_proxi_[i] = proxi;
  }

  // create IMUs, might consider using fake_imus based on input arguments
  for (int i = 0; i < data::Sensors::kNumImus; i++) {
    imu_[i] = new MPU9250(log_, chip_select_[i], true, 0x0);
  }
}

void Main::run()
{
  while (1) {
    // keep updating data_ based on values read from sensors

    // update BMS LP
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      bms_[i]->getData(&batteries_.low_power_batteries[i]);
    }

    // update front cluster of proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      proxi_[i]->getData(&sensors_.proxi_front[i]);
    }

    // update back cluster of proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      can_proxi_[i]->getData(&sensors_.proxi_back[i]);
    }

    // update imus
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i]->getData(&sensors_.imu[i]);
    }

    data_.setSensorsData(sensors_);
    data_.setBatteryData(batteries_);
    sleep(1000);
  }
}

}}  // namespace hyped::sensors
