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

#ifdef PROXI
#include "sensors/can_proxi.hpp"


#include "utils/system.hpp"
#include "utils/math/statistics.hpp"
#include "utils/concurrent/thread.hpp"

namespace hyped {

using utils::math::OnlineStatistics;
using utils::concurrent::Thread;

namespace sensors {


bool    CanProxi::can_registered_ = false;
bool    CanProxi::valid_[kNumProximities] = {};
uint8_t CanProxi::data_[kNumProximities] = {};
static_assert(kNumProximities <= 8, "cannot use more than 8 proxies over CAN");

CanProxi::CanProxi(uint8_t id, Logger& log)
    : log_(log),
      id_(id)
{
  if (!can_registered_) {
    for (auto& v : valid_) {
      v = false;
    }

    utils::io::Can::getInstance().registerProcessor(this);
    can_registered_ = true;
  }
}

// ---------------------------------------------------------------------
// ProxiInterface
// ---------------------------------------------------------------------
bool CanProxi::isOnline()
{
  return valid_[id_];
}
void CanProxi::getData(Proximity* proxi)
{
  proxi->val = data_[id_];
  proxi->operational = isOnline();
}

void CanProxi::startRanging() {}

// ---------------------------------------------------------------------
// CanProcessor
// ---------------------------------------------------------------------
void CanProxi::processNewData(Frame& message)
{
  switch (message.id) {
    case 0x45A: {   // echo reply
      // TODO(anyone): verify reply has the correct values (for debugging)
      break;
    }
    case 0x45B: {   // data message
      uint8_t size = kNumProximities < message.len ? kNumProximities : message.len;
      for (uint8_t i = 0; i < size; i++) {
        data_[i] = message.data[i];
      }
      break;
    }
    case 0x45C: {   // proxi health status
      uint8_t is_online = message.data[0];
      for (uint8_t i = 0; i < kNumProximities; i++) {
        valid_[i] = is_online & (1 << i);
      }
      break;
    }
    default: {
      log_.ERR("CanProxi", "received unsupported message with id %u", message.id);
    }
  }
}

bool CanProxi::hasId(uint32_t id, bool extended)
{
  switch (id) {
    case 0x45A:
    case 0x45B:
    case 0x45C:
      return true;
    default:
      return false;
  }
}


}}  // namespace hyped::sensors
#endif
