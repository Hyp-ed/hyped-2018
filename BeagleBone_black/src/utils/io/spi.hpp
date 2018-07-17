/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 18. April 2018
 * Description:
 * SPI abstracts the communication interface. The class implements a Singleton design pattern.
 *
 * Reads and Writes are blocking, i.e. SPI communication has completed by the time the read()
 * function returns.
 * The operations are not thread-safe. It is assumed only one thread uses SPI, namely IMU manager.
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

#ifndef BEAGLEBONE_BLACK_UTILS_IO_SPI_HPP_
#define BEAGLEBONE_BLACK_UTILS_IO_SPI_HPP_


#include "utils/logger.hpp"
#include "utils/utils.hpp"

namespace hyped {
namespace utils {
namespace io {


class SPI {
 public:
  static SPI& getInstance();

  enum class Clock {
    k1MHz,
    k4MHz,
    k20MHz
  };

  void setClock(Clock clk);

  /**
   * @brief simultaneous write and read. Write and read buffer should have the same length
   * @param tx  - pointer to head of write buffer
   * @param rx  - pointer to head of read  buffer
   * @param len - number of BYTES in each buffer
   */
  void transfer(uint8_t* tx, uint8_t* rx, uint16_t len);

  /**
   * @brief Get data from sensor, starting at some address.
   * @param addr  - register from which the reading should start
   * @param rx    - pointer to head of read buffer
   * @param len   - number of BYTES to be read, i.e. size of the read buffer
   */
  void read(uint8_t addr, uint8_t* rx, uint16_t len);

  /**
   * @brief Write data to sensor, starting at some address.
   * @param addr  - register from which writing to starts
   * @param tx    - pointer to head of write buffer
   * @param len   - number of BYTES to be written, i.e. size of the write buffer
   */
  void write(uint8_t addr, uint8_t* tx, uint16_t len);

 private:
  explicit SPI(Logger& log);
  ~SPI();
  int spi_fd_;
  Logger& log_;

  NO_COPY_ASSIGN(SPI);
};


}}}   // namespace hyped::utils::io


#endif  // BEAGLEBONE_BLACK_UTILS_IO_SPI_HPP_
