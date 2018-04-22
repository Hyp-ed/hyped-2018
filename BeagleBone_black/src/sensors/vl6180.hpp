/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 18/04/18
 * Description: Main file for Vl6180
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

// List of Includes
#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace sensors {

class Vl6180: public Thread {
 public:
  /**
    *  @brief  Turns the sensor on and prepares it
    */
  void turnOn();
  /**
    *  @brief  Turns the sensor off
    */
  void turnOff();
  /**
    *  @brief  Returns the distance from the nearest object the sensor is facing
    *
    *  @return double Returns the distance to the nearest object
    */
  double getDistance();

 private:
  Vl6180(uint8_t id, Logger& log);
  /**
    *  @brief  Loops until the device is out of reset
    *
    *  @return bool Returns true after the device is fresh out of reset
    */
  bool waitDeviceBooted();
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
  bool on_;
};

}} //namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_VL6180_HPP_
