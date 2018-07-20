/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/07/18
 * Description: Main class for fake IMUs.
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

#ifndef BEAGLEBONE_BLACK_SENSORS_FAKE_BATTERIES_HPP_
#define BEAGLEBONE_BLACK_SENSORS_FAKE_BATTERIES_HPP_

#include <string>
#include <vector>

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"

namespace hyped {

using utils::Logger;
using data::Data;

namespace sensors {


class FakeBatteries : public BMSInterface {
 public:
  FakeBatteries(Logger& log, bool is_high_voltage, bool is_nominal);
  void getData(Battery* battery) override;
  bool isOnline() override;

 private:
  void init();
  bool checkTime();

  Data& data_;
  bool is_started_;
  bool is_high_voltage_;
  uint64_t ref_time_;

  uint16_t voltage_;
  int16_t current_;
  int8_t temperature_;
  uint8_t charge_;
  uint16_t low_voltage_cell_;
  uint16_t high_voltage_cell_;
};
}}    // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_BATTERIES_HPP_
