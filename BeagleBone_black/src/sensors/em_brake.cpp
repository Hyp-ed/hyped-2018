/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 18/07/18
 * Description: EM brakes to update data structure if EM brakes deploy
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


#include "sensors/em_brake.hpp"
#include "data/data.hpp"

namespace hyped {

using data::Data;
using utils::System;
using utils::io::GPIO;

namespace sensors {

data::EmergencyBrakes EmBrake::em_data_;

EmBrake::EmBrake(Logger& log, bool is_front)
    : sys_(System::getSystem()),
      data_(data::Data::getInstance()),
      gpio_pin_(is_front ? 45 : 44, utils::io::gpio::kIn),
      is_front_(is_front)
{
  em_data_.module_status = data::ModuleStatus::kInit;
  data_.setEmergencyBrakesData(em_data_);
}

void EmBrake::run()
{
  int val;
  while (sys_.running_) {
    val = gpio_pin_.wait();
    if (is_front_) {
      em_data_.front_brakes = val;
    } else {
      em_data_.rear_brakes = val;
    }
    data_.setEmergencyBrakesData(em_data_);
  }
}
}}  // namespace hyped::sensors
