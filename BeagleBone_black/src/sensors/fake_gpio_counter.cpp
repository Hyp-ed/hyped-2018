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
      is_started_(false)
{}

void FakeGpioCounter::run()
{
  readDataFromFile(file_path_);
}

void FakeGpioCounter::readDataFromFile(std::string file_path)
{
    uint32_t timestamp = kTimeStamp;
    std::vector<uint64_t>* val_read = &val_read_;

    std::ifstream file;
    file.open(file_path);
    if (!file.is_open()) {
        log_.ERR("Fake-keyence", "Wrong file path for argument");
    }

    uint32_t temp_value;
    int counter = 0;
    uint32_t temp_time;
    std::string line;

    while (getline(file, line)) {
      std::stringstream input(line);
      input >> temp_time;

      if (temp_time != timestamp*counter) {
        log_.ERR("Fake-IMU-accl", "Timestamp format invalid %d", temp_time);
      }

      input >> temp_value;
      val_read->push_back(temp_value);
      counter++;
    }

    file.close();
}

void FakeGpioCounter::init()
{
  ref_time_ = utils::Timer::getTimeMicros();
  gpio_count_ = 0;
}

bool FakeGpioCounter::checkTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = now - ref_time_;

  if (time_span < kTimeStamp * gpio_count_) {
      return false;
  }
  gpio_count_ = time_span/kTimeStamp +1;
  return true;
}

StripeCounter FakeGpioCounter::getStripeCounter()
{
  data::StripeCounter stripes;
  if (data_.getStateMachineData().current_state == data::State::kAccelerating || is_started_) {
    if (!is_started_) {
      is_started_ = true;
      init();
    }
    if (checkTime()) {
      gpio_count_ = std::min(gpio_count_/kTimeStamp, (uint64_t) val_read_.size());
      if (gpio_count_ == (uint64_t) val_read_.size()) {
          prev_gpio_ = val_read_[gpio_count_-1];
      } else {
          prev_gpio_ = val_read_[gpio_count_];
      }
    }

    stripes.count.value = prev_gpio_;
    stripes.count.timestamp = ref_time_;
    stripes.operational = true;
  } else {
    stripes.count.value = 0;
    stripes.count.timestamp = 0;
    stripes.operational = true;
  }
  return stripes;
}

}}  // namespace hyped::sensors
