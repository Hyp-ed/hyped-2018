/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 20/06/18
 * Description: BMS manager for getting battery data
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


#include "sensors/bms_manager.hpp"

#include "sensors/bms.hpp"
#include "data/data.hpp"
#include "utils/timer.hpp"

namespace hyped {

using data::Batteries;

namespace sensors {

BmsManager::BmsManager(Logger& log, array<Battery, data::Batteries::kNumLPBatteries> *batteries)
    : ManagerInterface(log)
{
  // create BMS LP
  for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
    BMS* bms = new BMS(i, log_);
    bms->start();
    bms_[i] = bms;
  }

  lp_batteries_ = batteries;
}

void BmsManager::run()
{
  while (1) {
    // keep updating data_ based on values read from sensors
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      bms_[i]->getData(&((*lp_batteries_)[i]));
    }
    timestamp = utils::Timer::getTimeMicros();
    sleep(100);
  }
}

bool BmsManager::updated()
{
  if (old_timestamp_ != timestamp) {
    return true;
  }
  return false;
}

void BmsManager::resetTimestamp()
{
  old_timestamp_ = timestamp;
}
}}  // namespace hyped::sensors
