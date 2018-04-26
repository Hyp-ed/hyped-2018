/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 21. February 2018
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

#ifndef BEAGLEBONE_BLACK_UTILS_CONCURRENT_THREAD_HPP_
#define BEAGLEBONE_BLACK_UTILS_CONCURRENT_THREAD_HPP_

#include <cstdint>
#include <thread>
#include "utils/logger.hpp"

namespace hyped {
namespace utils {
namespace concurrent {


class Thread {
 public:
  explicit Thread(uint8_t id, Logger& log);
  // some constructors if you do not want to specify all the details
  explicit Thread(uint8_t id);
  explicit Thread(Logger& log);
  Thread();
  virtual ~Thread();

  /**
   * @brief      Spawn new thread and call Run() method
   */
  void start();

  /**
   * @brief      Wait until the thread terminates
   */
  void join();

  /**
   * @brief      Thread entry point
   */
  virtual void run();

  static void yield();

  uint8_t getId() { return id_; }

  static void sleep(uint32_t ms);

 private:
  uint8_t id_;
  std::thread* thread_;

 protected:
  Logger& log_;
};

class BusyThread : public Thread {
 public:
  ~BusyThread();
  void run() override;
  bool running_ = true;
};

}}}   // namespace hyped::utils::concurrent

#endif  // BEAGLEBONE_BLACK_UTILS_CONCURRENT_THREAD_HPP_
