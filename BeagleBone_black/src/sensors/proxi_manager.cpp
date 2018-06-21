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

namespace hyped {

using data::Data;
using data::Sensors;

namespace sensors {

ProxiManager::ProxiManager(Logger& log, bool isFront)
    : Thread(log),
      old_proxi_timestamp_(0)
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
    uint32_t time = utils::Timer::getTimeMicros();
    sensors_proxi_->timestamp = time;
  }
  sleep(10);
}

void ProxiManager::config(data::DataPoint<array<Proximity, data::Sensors::kNumProximities>> *proxi)
{
  sensors_proxi_ = proxi;
}

bool ProxiManager::updated()
{
  if (old_proxi_timestamp_ != sensors_proxi_->timestamp) {
    old_proxi_timestamp_ = sensors_proxi_->timestamp;
    return true;
  }
  return false;
}
}}  // namespace hyped::sensors
