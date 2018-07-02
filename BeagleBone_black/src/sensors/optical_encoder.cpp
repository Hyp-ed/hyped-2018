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
#include "sensors/optical_encoder.hpp"

#include <stdio.h>

#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/system.hpp"
#include "utils/timer.hpp"


namespace hyped {
using data::StripeCounter;
using utils::concurrent::Thread;
using utils::io::GPIO;

namespace sensors {

OpticalEncoder::OpticalEncoder(Logger& log, int pin)
    : Thread(log), pin_(pin)
{}

void OpticalEncoder::run()
{
  GPIO thepin(pin_, utils::io::gpio::kIn);
  uint8_t val = thepin.wait();  // Ignore first reading
  stripe_counter_.count.value = 0;
  stripe_counter_.count.timestamp =  utils::Timer::getTimeMicros();

  while (1) {
    val = thepin.wait();
    if (val == 1) {
      stripe_counter_.count.value = stripe_counter_.count.value+1;
      stripe_counter_.count.timestamp =  utils::Timer::getTimeMicros();
    }
  }
}

StripeCounter OpticalEncoder::getOptStripeCounter()
{
  return stripe_counter_;
}

}}  // namespace hyped::sensors
