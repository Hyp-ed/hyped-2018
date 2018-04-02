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
#endif	// BEAGLEBONE_BLACK_UTILS_IO_I2C_HPP_

#include "utils/concurrent/thread.hpp"

using hyped::utils::concurrent::Thread;

namespace hyped {
namespace utils {
namespace io {

class I2c : public Thread {
 public:
 I2c(uint8_t id);
 private:
  void run() override;
};
      
}}} // namespace hyped::utils::io
