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
#include "utils/timer.hpp"

namespace hyped {
namespace sensors {

std::vector<uint8_t> BMS::existing_ids_;    // NOLINT [build/include_what_you_use]
int16_t BMS::current_ = 0;
BMS::BMS(uint8_t id, Logger& log)
    : Thread(log),
      data_({}),
      id_(id),
      id_base_(bms::kIdBase + (bms::kIdIncrement * id_)),
      last_update_time_(0),
      can_(Can::getInstance()),
      running_(false)
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
  can_.registerProcessor(this);
  can_.start();

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

bool BMS::hasId(uint32_t id, bool extended)
{
  if (!extended) return false;  // this BMS only understands extended IDs

  // LP BMS CAN messages
  if (id_base_ <= id && id < id_base_ + bms::kIdSize) return true;

  // LP current CAN message
  if (id == 0x28) return true;

  return false;
}

void BMS::processNewData(utils::io::can::Frame& message)
{
  log_.DBG1("BMS", "module %u: received CAN message with id %d", id_, message.id);

  // check current CAN message
  if (message.id == 0x28) {
    if (message.len < 3) {
      log_.ERR("BMS", "module %u: current reading not enough data", id_);
      return;
    }

    current_ = (message.data[1] << 8) | (message.data[2]);
    return;
  }

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

  last_update_time_ = utils::Timer::getTimeMicros();
}


bool BMS::isOnline()
{
  // consider online if the data has been updated in the last second
  return (utils::Timer::getTimeMicros() - last_update_time_) < 1000000;
}

void BMS::getData(Battery* battery)
{
  battery->voltage = 0;
  for (uint16_t v: data_.voltage) battery->voltage += v;
  battery->voltage    /= 100;  // scale to 0.1V
  battery->temperature = data_.temperature;
  battery->current     = (-1*current_)/2;

  // charge calculation
  if (battery->voltage > 240) {                                       // constant high
    battery->charge = 95;
  } else if (240 >= battery->voltage && battery->voltage >= 180) {    // linear high
    battery->charge = battery->voltage / 0.75 - 225;
  } else if (180 >= battery->voltage && battery->voltage >= 150) {    // linear low
    battery->charge = battery->voltage / 2 - 75;
  } else {                                                            // constant low
    battery->charge = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// BMSHP
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<uint16_t> BMSHP::existing_ids_;   // NOLINT [build/include_what_you_use]

BMSHP::BMSHP(uint16_t id, Logger& log)
    : log_(log),
      can_id_(id*2 + bms::kHPBase),
      local_data_ {},
      last_update_time_(0)
{
  // verify this BMSHP unit has not been instantiated
  for (uint16_t i : existing_ids_) {
    if (id == i) {
      log_.ERR("BMSHP", "BMSHP %d already exists, duplicate unit instantiation", id);
      return;
    }
  }
  existing_ids_.push_back(id);

  // tell CAN about yourself
  Can::getInstance().registerProcessor(this);
  Can::getInstance().start();
}

bool BMSHP::isOnline()
{
  // consider online if the data has been updated in the last second
  return (utils::Timer::getTimeMicros() - last_update_time_) < 1000000;
}

void BMSHP::getData(Battery* battery)
{
  *battery = local_data_;
}

bool BMSHP::hasId(uint32_t id, bool extended)
{
  // only accept a single CAN message
  return id == can_id_ || id == static_cast<uint16_t>(can_id_ + 1);
}

void BMSHP::processNewData(utils::io::can::Frame& message)
{
  // message format is expected to look like this:
  // [ voltageH , volageL  , currentH   , currentL,
  //  charge   , HighTemp , AverageTemp, state, lowVoltageCellH,
  //  lowVoltageCellL ]
  if (message.id == can_id_) {
    local_data_.voltage     = (message.data[0] << 8) | message.data[1];
    local_data_.current     = (message.data[2] << 8) | message.data[3];
    local_data_.charge      = message.data[4] * 0.5;    // data needs scaling
    local_data_.temperature = message.data[5];
    local_data_.low_voltage_cell  = ((message.data[6] << 8) | message.data[7])/10;
    last_update_time_ = utils::Timer::getTimeMicros();
  } else {
    local_data_.high_voltage_cell = ((message.data[0] << 8) | message.data[1])/10;
  }

  log_.DBG1("BMSHP", "received data Volt,Curr,Char,Temp %u,%u,%u,%d",
    local_data_.voltage,
    local_data_.current,
    local_data_.charge,
    local_data_.temperature);
}




}}  // namespace hyped::sensors
