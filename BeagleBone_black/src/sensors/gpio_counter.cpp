/*
 * Author: Ragnor Comerford and Jack Horsburgh
 * Organisation: HYPED
 * Date: 186/18
 * Description: Main interface for IMU class.
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

#include <stdio.h>
#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/system.hpp"
#include "utils/timer.hpp"
#include "sensors/gpio_counter.hpp"



namespace hyped {

using data::StripeCounter;
using utils::concurrent::Thread;
using utils::io::GPIO;

namespace sensors {

GpioCounter::GpioCounter(int pin)
     : pin_(pin)
{}

void GpioCounter::run()
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
      stripe_counter_.operational = true;
    }
  }
}

StripeCounter GpioCounter::getStripeCounter()
{
  return stripe_counter_;
}

}}  // namespace hyped::sensors
