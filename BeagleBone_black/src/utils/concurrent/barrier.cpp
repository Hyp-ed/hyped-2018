/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 28. February 2018
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

#include "utils/concurrent/barrier.hpp"


namespace hyped {
namespace utils {
namespace concurrent {

Barrier::Barrier(uint8_t required)
    : required_(required),
      calls_(0)
{ /* EMPTY */ }

Barrier::~Barrier() { /* EMPTY */ }

void Barrier::wait()
{
  ScopedLock L(&lock_);
  calls_++;

  if (calls_ != required_) {
    cv_.wait(&lock_);
  } else {
    calls_ = 0;
    cv_.notifyAll();
  }
}

}}}   // namespace hyped::utils::concurrent

