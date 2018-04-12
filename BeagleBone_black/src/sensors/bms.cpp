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

bool BMS::exists_ = false;


BMS::BMS(): BMS(utils::System::getLogger())
{ /* Do nothing, delegate to the other constructor */ }

BMS::BMS(Logger& log)
    : log_(log)
    , can_(Can::getInstance())
{
  if (exists_) {
    log_.ERR("BMS", "BMS already exists, double module instantiation\n");
    return;
  }

  exists_ = true;

  // send initialisation CanFrame
  utils::io::CanFrame message;
  message.id = 300 | CAN_EFF_FLAG;
  message.len = 4;
  can_.send(message);

  log_.INFO("BMS", "init message sent\n");
}

}}  // namespace hyped::sensors
