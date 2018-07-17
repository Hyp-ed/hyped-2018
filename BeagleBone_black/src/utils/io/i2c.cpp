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

// #include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifndef WIN
// #include <linux/types.h>
#include <linux/i2c-dev.h>
#else
#define I2C_SLAVE 0x0703
#endif



#include "utils/logger.hpp"

#include "utils/system.hpp"

// #define I2C_ID 17

namespace hyped {
namespace utils {
namespace io {

namespace i2c {
inline int readHelper(int fd, uint8_t* buf, uint16_t len)
{
  return read(fd, buf, len);
}
inline int writeHelper(int fd, uint8_t* buf, uint16_t len)
{
  return write(fd, buf, len);
}
}   // namespace i2c


I2C& I2C::getInstance()
{
  static I2C i2c(System::getLogger());
  return i2c;
}

I2C::I2C(Logger& log)
    : log_(log),
      fd_(0),
      sensor_addr_(0)
{
  char device_name[] = "/dev/i2c-2";
  fd_ = open(device_name, O_RDWR, 0);
  if (fd_ < 0) log_.ERR("I2C", "Could not open i2c device");
}

I2C::~I2C()
{
  if (fd_ >= 0) close(fd_);
}

void I2C::setSensorAddress(uint32_t addr)
{
  if (fd_ < 0) return;

  sensor_addr_ = addr;
  int ret = ioctl(fd_, I2C_SLAVE, addr);
  if (ret < 0) log_.ERR("I2C", "Could not set sensor address");
}

bool I2C::read(uint32_t addr, uint8_t* rx, uint16_t len)
{
  if (fd_ < 0) return false;  // early exit if no i2c device present

  if (sensor_addr_ != addr) setSensorAddress(addr);

  int ret = i2c::readHelper(fd_, rx, len);
  return ret == len;
}

bool I2C::write(uint32_t addr, uint8_t* tx, uint16_t len)
{
  if (fd_ < 0) return false;  // early exit if no i2c device present

  if (sensor_addr_ != addr) setSensorAddress(addr);

  int ret = i2c::writeHelper(fd_, tx, len);
  return ret == len;
}

bool I2C::write(uint32_t addr, uint8_t tx)
{
  return write(addr, &tx, 1);
}

}}}   // namespace hyped::utils::io
