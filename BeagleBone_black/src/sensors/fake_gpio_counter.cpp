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
    : log_(log),
      data_(Data::getInstance()),
      ref_time_(0),
      timeout_(5000000),  // 5 seconds
      miss_stripe_(miss_stripe),
      double_stripe_(double_stripe),
      is_accelerating_(false)
{
  stripes_.operational = true;
  stripes_.count.value = 0;
}

StripeCounter FakeGpioCounter::getStripeCounter()
{
  data::Navigation nav   = data_.getNavigationData();
  data::State      state = data_.getStateMachineData().current_state;
  uint32_t prev_count    = stripes_.count.value;

  // create reference time only at transitions to dynamic states
  if (state == data::State::kAccelerating && !is_accelerating_) {
    ref_time_ = utils::Timer::getTimeMicros();
    is_accelerating_ = true;
  } else if (state == data::State::kDecelerating && is_accelerating_) {
    ref_time_ = utils::Timer::getTimeMicros();
    is_accelerating_ = false;
  }

  uint32_t new_count = std::floor(nav.distance/30.48);
  log_.DBG2("FAKE_GPCNTR", "nav.distance=%f, new_count=%d", nav.distance, new_count);

  switch (state) {
    case data::State::kAccelerating:
      if (timeout() && miss_stripe_) {
        new_count--;
      }
      break;
    case data::State::kDecelerating:
      if (timeout() && double_stripe_) {
        new_count++;
      }
      break;
    default:
      break;
  }

  if (new_count > prev_count) {
    stripes_.count.value     = new_count;
    stripes_.count.timestamp = utils::Timer::getTimeMicros();
  }
  log_.DBG2("FAKE_GPCNTR", "Returning count %d", stripes_.count.value);
  return stripes_;
}

bool FakeGpioCounter::timeout()
{
  return ref_time_ + timeout_ < utils::Timer::getTimeMicros();
}

}}  // namespace hyped::sensors
