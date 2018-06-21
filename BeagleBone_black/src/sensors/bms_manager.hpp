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

#ifndef BEAGLEBONE_BLACK_SENSORS_BMS_MANAGER_HPP_
#define BEAGLEBONE_BLACK_SENSORS_BMS_MANAGER_HPP_

#include <cstdint>

#include "sensors/manager_interface.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace sensors {

class BmsManager: public Thread, public BmsManagerInterface  {
 public:
  explicit BmsManager(Logger& log);
  void run() override;
  void config(array<Battery, data::Batteries::kNumLPBatteries> *batteries) override;
  bool updated() override;

 private:
  array<Battery, data::Batteries::kNumLPBatteries> *lp_batteries_;
  BMSInterface*   bms_[data::Batteries::kNumLPBatteries];
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_BMS_MANAGER_HPP_
