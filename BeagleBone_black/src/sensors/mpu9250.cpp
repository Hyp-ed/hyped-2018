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
#include <cstdint>

#include "sensors/mpu9250.hpp"
#include "utils/logger.hpp"

namespace hyped {
namespace sensors {

MPU9250::MPU9250(Logger& log)
    : log_(log)
{
  log_.INFO("MPU9250", "Creating a sensor with id: %d", 1);
}

MPU9250::~MPU9250()
{
  log_.INFO("MPU9250", "Deconstructing sensor object");
}

}}   // namespace hyped::sensors
