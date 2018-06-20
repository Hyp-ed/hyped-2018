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

#include "utils/timer.hpp"

#include <sys/time.h>

namespace hyped {
namespace utils {

uint64_t Timer::time_start_ = Timer::getTimeMicros();

// uint64_t Timer::getTimeMillis()
// {
//   return getTimeMicros() / 1000;
// }

uint64_t Timer::getTimeMicros()
{
  timeval tv;
  if (gettimeofday(&tv, (struct timezone *)0) < 0) {
    return 0;
  }
  return (static_cast<uint64_t>(tv.tv_sec)* 1000000) + tv.tv_usec - time_start_;
}

Timer::Timer()
    : elapsed_(0),
      start_(0),
      stop_(0)
{ /* EMPTY */ }

void Timer::start()
{
  start_ = getTimeMicros();
  stop_  = 0;
}

void Timer::stop()
{
  stop_  = getTimeMicros();
  elapsed_ += stop_ - start_;
}

void Timer::reset()
{
  start_    = 0;
  stop_     = 0;
  elapsed_  = 0;
}

uint64_t Timer::getMicros() const
{
  return elapsed_;
}

double Timer::getMillis() const
{
  return static_cast<double>(elapsed_) * 1.0e-3;
}

double Timer::getSeconds() const
{
  return static_cast<double>(elapsed_) * 1.0e-6;
}

}}  // namespace hyped::utils
