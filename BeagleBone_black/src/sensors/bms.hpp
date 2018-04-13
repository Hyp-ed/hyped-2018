/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 12. April 2018
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

#ifndef BEAGLEBONE_BLACK_SENSORS_BMS_HPP_
#define BEAGLEBONE_BLACK_SENSORS_BMS_HPP_

#include <stdint.h>

#include <vector>

#include "utils/concurrent/thread.hpp"

namespace hyped {
// Forward declarations
namespace utils { class Logger; }
namespace utils { namespace io { class Can; } }
namespace utils { namespace io { class CanFrame; } }

namespace sensors {

using utils::Logger;
using utils::io::Can;
using utils::concurrent::Thread;

#define BMS_FREQ     2                 // in Hz
#define BMS_PERIOD   1000/BMS_FREQ  // in microseconds

#define BMS_ID_BASE  300   // Base for ids of CAN messages related to BMS
#define BMS_ID_INCR  10    // increment of base dependent on id_
#define BMS_ID_SIZE  5     // size of id-space of BMS-CAN messages
/**
 * Bases of IDs of CAN messagese are calculated as follows:
 * BASE = BMS_ID_BASE + (BMS_ID_INCR * id_)
 */

#define BMS_CELL_NUM  7
struct BMS_Data {
  uint16_t  voltage[BMS_CELL_NUM];
  uint8_t   temperature;
};

class BMS : public Thread {
  friend Can;

 public:
  explicit BMS(uint8_t id);
  BMS(uint8_t id, Logger& log);

  ~BMS();

  /**
   * @brief Implement virtual run() from Thread
   */
  void run() override;

  BMS_Data getData() const
  {
    return data_;
  }

  BMS_Data* getDataPointer()
  {
    return &data_;
  }

 private:
  /**
   * @brief Send request CAN message to update data
   */
  void request();

  /**
   * @brief To be called by CAN receive side. BMS processes received CAN
   * message and updates its local data
   *
   * @param message received CAN message to be processed
   */
  void processNewData(utils::io::CanFrame& message);

 private:
  Can&      can_;
  BMS_Data  data_;
  uint8_t   id_;
  bool      running_;

  static std::vector<uint8_t> existing_ids_;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_BMS_HPP_
