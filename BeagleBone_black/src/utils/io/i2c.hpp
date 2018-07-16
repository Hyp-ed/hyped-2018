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


#include <cstdint>

#include "utils/utils.hpp"

namespace hyped {
namespace utils {

// Forward Declarations
class Logger;

namespace io {

class I2C {
 public:
  static I2C& getInstance();

  /**
   * @brief Get data from sensor.
   * @param addr  - sensor address
   * @param rx    - pointer to head of read buffer
   * @param len   - number of BYTES to be read, i.e. size of the read buffer
   * @return true - iff transaction was successful
   */
  bool read(uint32_t addr, uint8_t* rx, uint16_t len);

  /**
   * @brief Write data to sensor.
   * @param addr  - sensor address
   * @param rx    - pointer to head of write buffer
   * @param len   - number of BYTES to be written, i.e. size of the write buffer
   * @return true - iff transaction was successful
   */
  bool write(uint32_t addr, uint8_t* tx, uint16_t len);

  /**
   * @brief Write 1 BYTE to sensor.
   * @param addr  - sensor address
   * @param rx    - BYTE to be written
   * @return true - iff transaction was successful
   */
  bool write(uint32_t addr, uint8_t tx);

 private:
  explicit I2C(Logger& log);
  ~I2C();
  void setSensorAddress(uint32_t addr);

 private:
  Logger&   log_;
  int       fd_;
  uint32_t  sensor_addr_;

  NO_COPY_ASSIGN(I2C);
};

}}}   // namespace hyped::utils::io

#endif  // BEAGLEBONE_BLACK_UTILS_IO_I2C_HPP_
