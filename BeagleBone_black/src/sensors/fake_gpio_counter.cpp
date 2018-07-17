/*
 * Author: Jack Horsburgh and Ragnor Comerford
 * Organisation: HYPED
 * Date: 28/05/18
 * Description: Main class for fake gpio_counters.
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

#include "sensors/fake_gpio_counter.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "utils/timer.hpp"
#include "data/data.hpp"

namespace hyped {

using data::StripeCounter;

namespace sensors {

FakeGpioCounter::FakeGpioCounter(Logger& log, bool miss_stripe, bool double_stripe)
    : GpioInterface(log),
      data_(Data::getInstance()),
      miss_stripe_(miss_stripe),
      double_stripe_(double_stripe),
      is_started_(false)
{}

void FakeGpioCounter::run()
{
  while (1) {
    data::Navigation nav = data_.getNavigationData();
    data::State state = data_.getStateMachineData().current_state;
    uint32_t  prev_stripe = data_.getSensorsData().keyence_stripe_counter[0].count.value;
    uint64_t time_out = 5000000;
    if (state == data::State::kAccelerating && !is_started_) {
      init();
      is_started_ = true;
    }

    if (miss_stripe_) {
      if (time_out <= utils::Timer::getTimeMicros() - ref_time_) {
        if ((std::floor(nav.distance/30.48) - 1) == prev_stripe) {
           stripes_.count.value = std::floor(nav.distance/30.48) - 1;
        } else {
          stripes_.count.value = std::floor(nav.distance/30.48) - 1;
        }
      }
    } else if (double_stripe_) {
      if (time_out <= utils::Timer::getTimeMicros() - ref_time_) {
        stripes_.count.value = std::floor(nav.distance/30.48) + 1;
      }
    } else {
      stripes_.count.value = std::floor(nav.distance/30.48);
    }

    stripes_.count.timestamp = utils::Timer::getTimeMicros();
    stripes_.operational = true;
  }
  Thread::sleep(50);
}

void FakeGpioCounter::init()
{
  log_.INFO("Fake-Gpio", "TIMER START");
  ref_time_ = utils::Timer::getTimeMicros();
}

StripeCounter FakeGpioCounter::getStripeCounter()
{
  return stripes_;
}

}}  // namespace hyped::sensors
