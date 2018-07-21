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
#include "sensors/fake_batteries.hpp"

namespace hyped {
namespace sensors {

BmsManager::BmsManager(Logger& log,
                       BatteriesLP* lp_batteries,
                       BatteriesHP* hp_batteries)
    : ManagerInterface(log),
      lp_batteries_(lp_batteries),
      hp_batteries_(hp_batteries),
      sys_(utils::System::getSystem())
{
  old_timestamp_ = utils::Timer::getTimeMicros();
  // create BMS LP
  for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
    if (sys_.fake_batteries) {
      bms_[i] = new FakeBatteries(log, false, true);
    } else {
      BMS* bms = new BMS(i, log_);
      bms->start();
      bms_[i] = bms;
    }
  }
  for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
    if (sys_.fake_batteries) {
      bms_[i + data::Batteries::kNumLPBatteries] = new FakeBatteries(log, true, true);
    } else {
      bms_[i + data::Batteries::kNumLPBatteries] = new BMSHP(i, log_);
    }
  }
}

void BmsManager::run()
{
  while (1) {
    // keep updating data_ based on values read from sensors
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      bms_[i]->getData(&((*lp_batteries_)[i]));
      if (!bms_[i]->isOnline()) (*lp_batteries_)[i].voltage = 0;
    }
    for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
      bms_[i + data::Batteries::kNumLPBatteries]->getData(&(*hp_batteries_)[i]);
      if (!bms_[i + data::Batteries::kNumLPBatteries]->isOnline()) (*hp_batteries_)[i].voltage = 0;
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
