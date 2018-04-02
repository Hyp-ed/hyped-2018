/*
 * Authors: E. van Woerkom
 * Organisation: HYPED
 * Date: 28. March 2018
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
#include "i2c.hpp"
#include <iostream>

namespace hyped {
namespace utils {
namespace io {

void hyped::utils::io::I2c::run(){
  std::cout << "Started I2c thread" << std::endl;
  return;
}

I2c::I2c(uint8_t id) : concurrent::Thread(id) {}

}}} // namespace hyped::utils::io
