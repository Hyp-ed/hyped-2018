/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 02/07/18
 * Description: Driver for the OPB720B-12Z optical encoder
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

#ifndef BEAGLEBONE_BLACK_SENSORS_OPTICAL_ENCODER_HPP_
#define BEAGLEBONE_BLACK_SENSORS_OPTICAL_ENCODER_HPP_

#include <cstdint>

#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"
#include "data/data.hpp"


namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace sensors {

class OpticalEncoder: public Thread {
 public:
  OpticalEncoder(Logger& log, int pin);
  void run() override;
  data::StripeCounter getOptStripeCounter();

 private:
  int pin_;

  data::StripeCounter stripe_counter_;
};
}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_OPTICAL_ENCODER_HPP_
