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

#ifndef BEAGLEBONE_BLACK_SENSORS_FAKE_GPIO_COUNTER_HPP_
#define BEAGLEBONE_BLACK_SENSORS_FAKE_GPIO_COUNTER_HPP_

#include <string>
#include <vector>

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"

namespace hyped {

using utils::Logger;
using data::Data;

namespace sensors {


class FakeGpioCounter:public GpioInterface {
 public:
  FakeGpioCounter(Logger& log, bool miss_stripe, bool double_stripe);
  data::StripeCounter getStripeCounter() override;

 private:
  bool timeout();
  Logger&     log_;
  Data&       data_;

  uint64_t              ref_time_;
  uint64_t              timeout_;
  data::StripeCounter   stripes_;
  bool                  miss_stripe_;
  bool                  double_stripe_;
  bool                  is_accelerating_;
};
}}    // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_GPIO_COUNTER_HPP_
