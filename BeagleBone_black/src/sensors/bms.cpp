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

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/io/can.hpp"

namespace hyped {
namespace sensors {

std::vector<uint8_t> BMS::existing_ids_;    // NOLINT [build/include_what_you_use]


BMS::BMS(uint8_t id): BMS(id, utils::System::getLogger())
{ /* Do nothing, delegate to the other constructor */ }

BMS::BMS(uint8_t id, Logger& log)
    : Thread(log)
    , can_(Can::getInstance())
    , id_(id)
{
  // verify the module has not been instantiated
  for (uint8_t i : existing_ids_) {
    if (id == i) {
      log_.ERR("BMS", "BMS %d already exists, double module instantiation\n", id);
      return;
    }
  }
  existing_ids_.push_back(id);

  // reset module's data
  for (auto& v : data_.voltage) {
    v = 0;
  }
  data_.temperature = 0;

  // tell CAN about yourself
  can_.registerBMS(this);

  // spawn helper thread
  running_ = true;
  start();
}

BMS::~BMS()
{
  running_ = false;
  join();
}

void BMS::request()
{
  // send request CanFrame
  utils::io::CanFrame message;
  message.id        = 300;
  message.extended  = true;
  message.len       = 2;
  message.data[0]   = 0;
  message.data[1]   = 0;

  can_.send(message);
  log_.DBG1("BMS", "request message sent\n");
}

void BMS::run()
{
  log_.INFO("BMS", "starting BMS module %d\n", id_);
  while (running_) {
    request();
    sleep(BMS_PERIOD);
  }
  log_.INFO("BMS", "stopped BMS module %d\n", id_);
}

void BMS::processNewData(utils::io::CanFrame& message)
{
  // TODO(anybody): add message processing
  log_.DBG1("BMS", "id: %d, received CAN message with id %d\n", id_, message.id);
  log_.DBG2("BMS", "message data[0,1] %d %d\n", message.data[0], message.data[1]);
  uint8_t offset = message.id - (BMS_ID_BASE + (BMS_ID_INCR * id_));
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
      data_.temperature = message.data[0] - BMS_TEMP_OFFSET;
      break;
    default:
      log_.ERR("BMS", "received invalid message, id %d, offset %d\n"
        , message.id
        , offset);
  }
}

}}  // namespace hyped::sensors
