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

#include "sensors/bms.hpp"

#include "data/data.hpp"
#include "utils/io/can.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/utils.hpp"

namespace hyped {
namespace sensors {

std::vector<uint8_t> BMS::existing_ids_;    // NOLINT [build/include_what_you_use]

BMS::BMS(uint8_t id): BMS(id, 0, utils::System::getLogger())
{ /* Do nothing, delegate to the other constructor */ }

BMS::BMS(uint8_t id, data::Battery* battery_unit)
    : BMS(id, battery_unit, utils::System::getLogger())
{ /* Do nothing, delegate to the other constructor */ }

BMS::BMS(uint8_t id, Logger& log)
    : BMS(id, 0, log)
{ /* Do nothing, delegate to the other constructor */ }

BMS::BMS(uint8_t id, data::Battery* battery_unit, Logger& log)
    : Thread(log)
    , can_(Can::getInstance())
    , data_({})
    , battery_unit_(battery_unit)
    , id_(id)
    , running_(false)
{
  ASSERT(id < data::Batteries::kNumLPBatteries);
  // verify this BMS unit has not been instantiated
  for (uint8_t i : existing_ids_) {
    if (id == i) {
      log_.ERR("BMS", "BMS %d already exists, duplicate unit instantiation", id);
      return;
    }
  }
  existing_ids_.push_back(id);

  // tell CAN about yourself
  can_.registerBMS(this);

  running_ = true;
}

BMS::~BMS()
{
  running_ = false;
  join();
}

void BMS::request()
{
  // send request CanFrame
  utils::io::can::Frame message;
  message.id        = bms::kIdBase + (bms::kIdIncrement * id_);
  message.extended  = true;
  message.len       = 2;
  message.data[0]   = 0;
  message.data[1]   = 0;

  can_.send(message);
  log_.DBG1("BMS", "module %u: request message sent", id_);
}

void BMS::run()
{
  log_.INFO("BMS", "module %u: starting BMS", id_);
  while (running_) {
    request();
    sleep(bms::kPeriod);
  }
  log_.INFO("BMS", "module %u: stopped BMS", id_);
}

void BMS::processNewData(utils::io::can::Frame& message)
{
  log_.DBG1("BMS", "module %u: received CAN message with id %d", id_, message.id);
  log_.DBG2("BMS", "message data[0,1] %d %d", message.data[0], message.data[1]);
  uint8_t offset = message.id - (bms::kIdBase + (bms::kIdIncrement * id_));
  switch (offset) {
    case 0x1:   // cells 1-4
      for (int i = 0; i < 4; i++) {
        data_.voltage[i] = (message.data[2*i] << 8) | message.data[2*i + 1];
      }
      break;
    case 0x2:   // cells 5-7
      for (int i = 0; i < 3; i++) {
        data_.voltage[4 + i] = (message.data[2*i] << 8) | message.data[2*i + 1];
      }
      break;
    case 0x3:   // ignore, no cells connected
      break;
    case 0x4:   // temperature
      data_.temperature = message.data[0] - bms::Data::kTemperatureOffset;
      break;
    default:
      log_.ERR("BMS", "received invalid message, id %d, CANID %d, offset %d",
          id_, message.id, offset);
  }
}

void BMS::update()
{
  if (!battery_unit_) {
    log_.ERR("BMS", "module %u: does not have data::Battery pointer");
    return;
  }

  uint16_t voltage = 0;
  for (uint16_t v : data_.voltage) voltage += v;

  battery_unit_->voltage      = voltage;
  battery_unit_->temperature  = data_.temperature;
}

}}  // namespace hyped::sensors
