/*
 * Authors: E. van Woerkom
 * Organisation: HYPED
 * Date: 28. March 2018
 * Description:
 * Head for main i2c thread
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

#ifndef BEAGLEBONE_BLACK_UTILS_IO_I2C_HPP_
#define BEAGLEBONE_BLACK_UTILS_IO_I2C_HPP_

#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"

namespace hyped {
namespace utils {

using concurrent::Thread;

namespace io {

class I2C : public concurrent::Thread {
 public:
  I2C();
  ~I2C();
  void run() override;
};

}}}   // namespace hyped::utils::io

#endif  // BEAGLEBONE_BLACK_UTILS_IO_I2C_HPP_
