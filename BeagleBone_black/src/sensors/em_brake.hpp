/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 19/06/18
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

#ifndef BEAGLEBONE_BLACK_SENSORS_EM_BRAKE_HPP_
#define BEAGLEBONE_BLACK_SENSORS_EM_BRAKE_HPP_

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "utils/system.hpp"
#include "utils/io/gpio.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;
using utils::io::GPIO;

namespace sensors {

class EmBrake: public Thread {
 public:
  EmBrake(Logger& log, bool is_front);
  void run()    override;

 private:
  utils::System&    sys_;
  data::Data&       data_;
  GPIO gpio_pin_;
  static data::EmergencyBrakes em_data_;
  bool is_front_;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_EM_BRAKE_HPP_
