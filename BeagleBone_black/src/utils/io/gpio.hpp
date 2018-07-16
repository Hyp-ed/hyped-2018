/*
 * Authors: Martin Kristien
 * Organisation: HYPED
 * Date: 20. April 2018
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

#ifndef BEAGLEBONE_BLACK_UTILS_IO_GPIO_HPP_
#define BEAGLEBONE_BLACK_UTILS_IO_GPIO_HPP_


#include <cstdint>
#include <vector>

#include "utils/utils.hpp"

namespace hyped {
namespace utils {
// Forward declaration
class Logger;
namespace io {

namespace gpio {
constexpr uint8_t kBankNum = 4;
enum Direction {
  kIn   = 0,
  kOut  = 1
};
}   // namespace gpio

class GPIO {
 public:
  GPIO(uint32_t pin, gpio::Direction direction);
  GPIO(uint32_t pin, gpio::Direction direction, Logger& log);

  void    set();     // set high
  void    clear();   // set low
  uint8_t read();    // read pin value

  /**
   * @brief Block caller until value of gpio pin has changed
   * @return int8_t the new gpio value, -1 in case of an error
   */
  int8_t wait();

 private:
  GPIO() = delete;

  // GPIO system configuration
  /**
   * @brief Fill in base_mapping_ with pointers to mmap-ed /dev/mem
   * to 4 GPIO banks/ports.
   */
  static void initialise();
  static void uninitialise();
  static bool initialised_;
  static void* base_mapping_[gpio::kBankNum];
  static std::vector<uint32_t> exported_pins;

  /**
   * @brief Tell kernel we are using this, set direction
   */
  void exportGPIO();

  /**
   * @brief Configure device register pointers
   */
  void attachGPIO();

  /**
   * @brief Configure fd_ for wait functionality in case of input GPIO
   */
  void setupWait();

  uint32_t        pin_;
  gpio::Direction direction_;
  Logger&         log_;

  volatile uint32_t* set_;        // set register
  volatile uint32_t* clear_;      // clear register
  volatile uint32_t* data_;       // data register
  uint32_t           pin_mask_;   // mask for register access to this pin
  int                fd_;         // file pointer to /sys/class/gpio/gpioXX/value

  NO_COPY_ASSIGN(GPIO);
};

}}}   // namespace hyped::utils::io

#endif  // BEAGLEBONE_BLACK_UTILS_IO_GPIO_HPP_
