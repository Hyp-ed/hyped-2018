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

#ifndef BEAGLEBONE_BLACK_UTILS_CONCURRENT_CONDITION_VARIABLE_HPP_
#define BEAGLEBONE_BLACK_UTILS_CONCURRENT_CONDITION_VARIABLE_HPP_

#define CV  condition_variable_any

#include <condition_variable>

namespace hyped {
namespace utils {
namespace concurrent {

// Forward declaration
class Lock;

class ConditionVariable {
 public:
  ConditionVariable();
  ~ConditionVariable();

  /**
   * @brief      Wake up one thread waiting for this CV.
   */
  void notify();

  /**
   * @brief      Wake up all threads waiting for this CV.
   */
  void notifyAll();

  /**
   * @brief      Block current thread until this CV is signalled/notified.
   *
   * @param      lock  The lock associated with this CV. The lock is unlocked
   *                   upon calling this method and reacquired just before returning.
   *                   Note, the lock must be acquired by the caller before calling wait().
   */
  void wait(Lock* lock);

 private:
  std::CV* cond_var_;
};

}}}   // namespace hyped::utils::concurrent

#endif  // BEAGLEBONE_BLACK_UTILS_CONCURRENT_CONDITION_VARIABLE_HPP_
