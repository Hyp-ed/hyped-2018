/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 18/04/18
 * Description: Main file for VL6180
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

#ifndef BEAGLEBONE_BLACK_SENSORS_VL6180_HPP_
#define BEAGLEBONE_BLACK_SENSORS_VL6180_HPP_


#include "utils/logger.hpp"
#include "utils/io/i2c.hpp"

constexpr uint8_t DEFAULT_I2C_SLAVE_ADDR = 0x29;

namespace hyped {

using hyped::utils::io::I2C;
using utils::Logger;

namespace sensors {

class VL6180 {
 public:
  VL6180(uint8_t i2c_addr, Logger& log);
  ~VL6180();
  /**
    *  @brief  Returns the distance from the nearest object the sensor is facing
    *
    *  @return double Returns the distance to the nearest object
    */
  double getDistance();
  /**
    *  @brief  Sets the the ranging mode to continuous
    */
  void setContinuousRangingMode();
  /**
    *  @brief  Sets the the ranging mode to single shot
    */
  void setSingleShotMode();

 private:
  /**
    *  @brief called from getDistance() for single shot ranging
    *
    *  @return double Returns the distance to the nearest object
    */
  double singleRangeDistance();
  /**
    *  @brief called from getDistance() for continuous ranging
    *
    *  @return double Returns the distance to the nearest object
    */
  double continuousRangeDistance();
  /**
    *  @brief  Loops until the device is out of reset
    *
    *  @return bool Returns true after the device is fresh out of reset
    */
  bool waitDeviceBooted();
  /**
    *  @brief  Turns the sensor off
    */
  void turnOff();
  /**
    *  @brief  Turns the sensor on and prepares it
    */
  void turnOn();
  /**
    *  @brief  Sets the maximum convergence time in ms
    */
  void setMaxCovergenceTime(uint8_t time);
  /**
    *  @brief  Wait for sensor to be ready before a new ranging command
    *            is issued
    *
    *  @return bool Returns true when device is ready to get a new range
    */
  bool rangeWaitDeviceReady();
  /**
    *  @brief  Reads a single byte register and returns its status
    *
    *  @return int Returns 0 if successful
    */
  int readByte(uint16_t reg_add, uint8_t *data);
  /**
    *  @brief  Writes a byte to the register and returns its status
    *
    *  @return int Returns 0 if successful
    */
  int writeByte(uint16_t reg_add, char data);
  Logger& log_;
  bool on_;
  bool continuous_mode_;
  uint8_t i2c_addr_ = DEFAULT_I2C_SLAVE_ADDR;
  I2C& i2c_ = I2C::getInstance();
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_VL6180_HPP_
