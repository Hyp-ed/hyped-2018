/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 19/06/18
 * Description: Proxi manager for getting proxi data from around the pod
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

#include "sensors/proxi_manager.hpp"

#include "sensors/can_proxi.hpp"
#include "sensors/vl6180.hpp"
#include "data/data.hpp"
#include "utils/timer.hpp"
#include "utils/io/i2c.hpp"

namespace hyped {

using data::Data;
using data::Sensors;
using utils::io::I2C;

namespace sensors {

ProxiManager::ProxiManager(Logger& log, bool isFront)
    : Thread(log)
{
  if (isFront) {
    // create CAN-based proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      CanProxi* proxi = new CanProxi(i, log_);
      proxi_[i] = proxi;
    }
  } else {
    I2C& i2c = I2C::getInstance();
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      i2c.write(kMultiplexerAddr, 1 << i);  // open particular i2c channel
      VL6180* proxi = new VL6180(0x29, log_);
      proxi->setContinuousRangingMode();
      proxi->setAddress(0x29 + i);
      proxi_[i] = proxi;
    }
    i2c.write(kMultiplexerAddr, 0xFF);      // open all i2c channels
  }
}

void ProxiManager::run()
{
  while (1) {
    // update front cluster of proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      proxi_[i]->getData(&(sensors_proxi_->value[i]));
    }
    uint32_t time = utils::Timer::getTimeMicros();
    sensors_proxi_->timestamp = time;
  }
  sleep(10);
}

void ProxiManager::config(data::DataPoint<array<Proximity, data::Sensors::kNumProximities>> *proxi)
{
  sensors_proxi_ = proxi;
}
}}  // namespace hyped::sensors
