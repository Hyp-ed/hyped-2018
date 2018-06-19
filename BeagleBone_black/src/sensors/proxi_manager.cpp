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

namespace hyped {

using data::Data;
using data::Sensors;

namespace sensors {

ProxiManager::ProxiManager(uint8_t id, Logger& log, bool isFront)
    : Thread(id, log),
      data_(data::Data::getInstance())
{
  if (isFront) {
    // create CAN-based proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      CanProxi* proxi = new CanProxi(i, log_);
      proxi_[i] = proxi;
    }
  } else {
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      VL6180* proxi = new VL6180(0x29, log_);
      proxi->setContinuousRangingMode();
      proxi_[i] = proxi;
    }
  }
}

void ProxiManager::run()
{
  while (1) {
    // update front cluster of proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    proxi_[i]->getData(&(sensors_proxi_->value[i]));
    }
  }
}

void ProxiManager::config(data::DataPoint<array<Proximity, data::Sensors::kNumProximities>> *proxi)
{
  sensors_proxi_ = proxi;
}
}}  // namespace hyped::sensors
