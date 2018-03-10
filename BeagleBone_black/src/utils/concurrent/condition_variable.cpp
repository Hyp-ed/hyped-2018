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

#include "utils/concurrent/condition_variable.hpp"

#include "utils/concurrent/lock.hpp"

namespace hyped {
namespace utils {
namespace concurrent {

ConditionVariable::ConditionVariable()
{
  cond_var_ = new std::CV();
}

ConditionVariable::~ConditionVariable()
{
  delete cond_var_;
}

void ConditionVariable::notify()
{
  cond_var_->notify_one();
}

void ConditionVariable::notifyAll()
{
  cond_var_->notify_all();
}

void ConditionVariable::wait(Lock* lock)
{
  cond_var_->wait(*lock->mutex_);
}

}}}   // hyped::utils::concurrent

