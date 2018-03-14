/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 27. February 2018
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

#include "utils/concurrent/lock.hpp"

namespace hyped {
namespace utils {
namespace concurrent {

Lock::Lock()
{
  mutex_ = new std::mutex();
}

Lock::~Lock()
{
  delete mutex_;
}

void Lock::lock()
{
  mutex_->lock();
}

bool Lock::tryLock()
{
  return mutex_->try_lock();
}

void Lock::unlock()
{
  return mutex_->unlock();
}

}}}   // namespace hyped::utils::concurrent
