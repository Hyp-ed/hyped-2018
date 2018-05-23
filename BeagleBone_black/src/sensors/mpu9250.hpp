/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 23/05/18
 * Description: Main file for MPU9250
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

#ifndef BEAGLEBONE_BLACK_SENSORS_MPU9250_HPP_
#define BEAGLEBONE_BLACK_SENSORS_MPU9250_HPP_

#include "utils/logger.hpp"
#include "utils/io/i2c.hpp"

namespace hyped {

using hyped::utils::io::I2C;
using utils::Logger;

namespace sensors {

class MPU9250 {
 public:
  // TODO(Jack)
  explicit MPU9250(Logger& log);
  ~MPU9250();

 private:
  // TODO(Jack)
  Logger& log_;
  I2C& i2c_ = I2C::getInstance();
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_MPU9250_HPP_
