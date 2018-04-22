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
#include "utils/io/i2c.hpp"

#define I2C_ID 17

namespace hyped {
namespace utils {
namespace io {

Logger i2clog(true, 1);

I2C::I2C() : concurrent::Thread(I2C_ID, i2clog)
{
  i2clog.INFO("I2C", "Constructing");
  return;
}

I2C::~I2C()
{
  i2clog.INFO("I2C", "Destructing");
  return;
}

void I2C::run()
{
  i2clog.INFO("I2C", "Running");
  return;
}

}}}   // namespace hyped::utils::io
