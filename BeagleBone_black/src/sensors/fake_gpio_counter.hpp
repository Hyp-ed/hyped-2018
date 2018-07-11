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

namespace sensors {


class FakeGpioCounter:public GpioInterface {
 public:
  explicit FakeGpioCounter(Logger& log, std::string file_path);
  data::StripeCounter getStripeCounter() override;
  void run() override;

 private:
  void readDataFromFile(std::string file_path);
  void init();
  bool checkTime();
  std::string file_path_;

  std::vector<uint64_t> val_read_;
  uint64_t gpio_count_;

  data::StripeCounter stripe_counter_;
  uint64_t ref_time_;
  uint64_t prev_gpio_;
  bool is_started_;
};
}}    // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_GPIO_COUNTER_HPP_
