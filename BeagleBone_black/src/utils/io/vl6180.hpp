/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
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

 #ifndef BEAGLEBONE_BLACK_UTILS_IO_VL6180_HPP_
 #define BEAGLEBONE_BLACK_UTILS_IO_VL6180_HPP_

 // List of Includes
#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"
#include "data/data.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace utils {
namespace io{



class Vl6180: public Thread {
  public:
    void turnOn();
    void turnOff();
    int getDistance();



  private:
    Vl6180(uint8_t id, Logger& log);
    data::Data& data = data::Data::getInstance();
    data::StateMachine state;
    bool waitDeviceBooted();
    bool rangeWaitDeviceReady();
    int readByte(uint16_t reg_add, uint8_t *data);
    int writeByte(uint16_t reg_add, char data);
    bool on;
};

}}} //namespace hyped::utils::io

 #endif  // BEAGLEBONE_BLACK_UTILS_IO_VL6180_HPP_