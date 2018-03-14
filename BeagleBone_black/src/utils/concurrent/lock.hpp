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

#ifndef BEAGLEBONE_BLACK_UTILS_CONCURRENT_LOCK_HPP_
#define BEAGLEBONE_BLACK_UTILS_CONCURRENT_LOCK_HPP_


#include <mutex>
// lock, try_lock, unlock in mutex, unique_lock

// condition_variable[_any] -> notify_[one/all], wait


namespace hyped {
namespace utils {
namespace concurrent {

// Forward declaration
class ConditionVariable;

class Lock {
  friend ConditionVariable;

 public:
  Lock();
  ~Lock();

  /**
   * @brief      Acquire the associated lock. Blocking.
   */
  void lock();

  /**
   * @brief      Try to acquire the associated lock. Non-blocking.
   *
   * @return     True iff you own the lock now.
   */
  bool tryLock();

  /**
   * @brief      Releases the ownership of the lock. Should only be called
   *             if you own the lock.
   */
  void unlock();

 private:
  std::mutex* mutex_;
};

class ScopedLock {
 public:
  explicit ScopedLock(Lock* lock)
    : lock_(lock)
  {
    lock_->lock();
  }
  ~ScopedLock()
  {
    lock_->unlock();
  }
 private:
  Lock* lock_;
};

}}}   // namespace hyped::utils::concurrent


#endif  // BEAGLEBONE_BLACK_UTILS_CONCURRENT_LOCK_HPP_
