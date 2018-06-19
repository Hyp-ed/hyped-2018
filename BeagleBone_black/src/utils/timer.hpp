/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 18. April 2018
 * Description:
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

#ifndef BEAGLEBONE_BLACK_UTILS_TIMER_HPP_
#define BEAGLEBONE_BLACK_UTILS_TIMER_HPP_

#include <stdint.h>

#include "utils/utils.hpp"

namespace hyped {
namespace utils {

class Timer {
 public:
  // static uint64_t getTimeMillis();
  static uint64_t getTimeMicros();

  Timer();

  void start();
  void stop();
  void reset();
  double getSeconds() const;
  double getMillis() const;
  uint64_t getMicros() const;

 private:
  uint64_t elapsed_;
  uint64_t start_;
  uint64_t stop_;
  static uint64_t time_start_;
  NO_COPY_ASSIGN(Timer);
};

class ScopedTimer {
 public:
  explicit ScopedTimer(Timer* t)
      : timer_(t)
  {
    timer_->start();
  }

  ~ScopedTimer()
  {
    timer_->stop();
  }

 private:
  Timer* timer_;
  NO_COPY_ASSIGN(ScopedTimer);
};

}}  // namespace hyped::utils

#endif  // BEAGLEBONE_BLACK_UTILS_TIMER_HPP_
