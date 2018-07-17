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

// TODO(Jack) Find the least between stripes
constexpr uint8_t kTimeStamp = 50;    // ms

namespace hyped {

using data::StripeCounter;

namespace sensors {

FakeGpioCounter::FakeGpioCounter(Logger& log, std::string file_path)
    : GpioInterface(log),
      data_(Data::getInstance()),
      file_path_(file_path),
      is_started_(false),
      prev_gpio_(0)
{}

void FakeGpioCounter::run()
{
  data::Navigation nav = data_.getNavigationData();
  data::State state = data_.getStateMachineData().current_state;
  if (state == data::State::kAccelerating) {
    init();
  }

  if (checkTime()) {
    stripes_.count.value = std::floor(nav.distance/30.48);
    stripes_.count.timestamp = utils::Timer::getTimeMicros();
    stripes_.operational = true;
  }
}

void FakeGpioCounter::init()
{
  log_.INFO("Fake-Gpio", "TIMER START");
  ref_time_ = utils::Timer::getTimeMicros();
  gpio_count_ = 0;
}

bool FakeGpioCounter::checkTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = (now - ref_time_) / 1000;

  if (time_span < kTimeStamp * gpio_count_) {
      return false;
  }
  gpio_count_ = time_span/kTimeStamp + 1;
  // log_.INFO("Fake-Gpio", "Gpio-count: %d", gpio_count_);
  return true;
}

StripeCounter FakeGpioCounter::getStripeCounter()
{
  return stripes_;
}

}}  // namespace hyped::sensors
