/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 12. April 2018
 * Description:
 * Battery Management System abstraction. Each object corresponds to one low-powered BMS unit.
 * Each unit is identified by a unique ID, which is also used to infer IDs of CAN messages.
 *
 * Configuration and local data structure are encapsulated in namespace bms.
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

#include <cstdint>
#include <vector>

#include "utils/concurrent/thread.hpp"

namespace hyped {
// Forward declarations
namespace utils { class Logger; }
namespace utils { namespace io { class Can; } }
namespace utils { namespace io { namespace can { struct Frame; } } }

namespace sensors {

using utils::Logger;
using utils::io::Can;
using utils::concurrent::Thread;

namespace bms {
// how often shall request messages be sent
constexpr uint32_t kFreq    = 2;           // in Hz
constexpr uint32_t kPeriod  = 1000/kFreq;  // in milliseconds

// what is the CAN ID space for BMS units
constexpr uint16_t kIdBase      = 300;     // Base for ids of CAN messages related to BMS
constexpr uint16_t kIdIncrement = 10;      // increment of base dependent on id_
constexpr uint16_t kIdSize      = 5;       // size of id-space of BMS-CAN messages
/**
 * Bases of IDs of CAN messagese for a BMS unit are calculated as follows:
 * base = kIdBase + (kIdIncrement * id_)
 */

struct Data {
  static constexpr uint8_t kTemperatureOffset = 40;
  static constexpr uint8_t kCellNum           = 7;
  uint16_t  voltage[kCellNum];
  uint8_t   temperature;
};

}   // namespace bms

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

  bms::Data getData() const { return data_; }
  bms::Data* getDataPointer() { return &data_; }

 private:
  /**
   * @brief Send request CAN message to update data periodically
   */
  void request();

  /**
   * @brief To be called by CAN receive side. BMS processes received CAN
   * message and updates its local data
   *
   * @param message received CAN message to be processed
   */
  void processNewData(utils::io::can::Frame& message);

 private:
  Can&      can_;
  bms::Data data_;
  uint8_t   id_;
  bool      running_;

  static std::vector<uint8_t> existing_ids_;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_BMS_HPP_
