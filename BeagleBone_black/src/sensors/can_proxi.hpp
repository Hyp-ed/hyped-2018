/*
 * Author: Martin Kristien
 * Organisation: HYPED
 * Date: 08/06/18
 * Description: Implements proximity driver over CANBUS using
 * data from SAMC21 board
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

#ifndef BEAGLEBONE_BLACK_SENSORS_CAN_PROXI_HPP_
#define BEAGLEBONE_BLACK_SENSORS_CAN_PROXI_HPP_
#ifdef PROXI

#include <cstdint>

#include "data/data.hpp"
#include "sensors/interface.hpp"
#include "utils/io/can.hpp"
#include "utils/system.hpp"

namespace hyped {

// namespace utils { namespace io { class Can; }}
namespace utils { class Logger; }

using utils::io::can::Frame;
using utils::io::CanProccesor;
using utils::Logger;

namespace sensors {

constexpr uint32_t kNumProximities = data::Sensors::kNumProximities;


/**
 * @brief Represents and instance of a proximity sensor connected to SAMC21 board.
 * The values are transfered over CANBUS.
 *
 * Note, all 8 values are transfered as a single CAN message with a single ID.
 * Therefore, only 1 object handles CAN message processing and puts the data to
 * the static data[] variable for all other proximity objects to share.
 */
class CanProxi : public ProxiInterface, public CanProccesor {
 public:
  CanProxi(uint8_t id, Logger& log = utils::System::getLogger());

  // from ProxiInterface
  bool isOnline() override;
  void getData(Proximity* proxi) override;

  // from CanProcessor
  void processNewData(Frame& message) override;
  bool hasId(uint32_t id, bool extended) override;
  void startRanging() override;
 private:
  Logger& log_;
  uint8_t id_;

  static bool    can_registered_;
  static bool    valid_[kNumProximities];
  static uint8_t data_[kNumProximities];
};

}}  // namespace hyped::sensors
#endif
#endif  // BEAGLEBONE_BLACK_SENSORS_CAN_PROXI_HPP_

