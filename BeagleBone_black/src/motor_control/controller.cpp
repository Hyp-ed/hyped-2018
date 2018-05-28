/*
 * Author: Sean Mullan
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * A controller object represents one motor controller. Object is used to request data specific to
 * the controller.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "motor_control/controller.hpp"
#include <cstdint>

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/io/can.hpp"

namespace hyped {
namespace motor_control {

using hyped::utils::io::can::Frame;

constexpr uint16_t sdo_receive = 0x600;

// Status masks, used to mask the return result of a CAN message
// pg51 CANOpen_Motion_Control.pdf
constexpr uint16_t kNotReadyToSwitchMask       = 0x0000;
constexpr uint16_t kSwitchOnDisabledMask       = 0x0040;
constexpr uint16_t kReadyToSwitchOnMask        = 0x0021;
constexpr uint16_t kSwitchedOnMask             = 0x0023;
constexpr uint16_t kOperationEnabledMask       = 0x0027;
constexpr uint16_t kQuickStopActiveMask        = 0x0007;
constexpr uint16_t kFaultReactionActiveMask    = 0x0008;
// Unsure about this check datasheet TODO(anyone)
constexpr uint16_t kFaultMask                  = 0x0008;

// Masks for profile velocity status
// pg123 'CANOpen_Motion_Control.pdf
constexpr uint16_t kTargetReachedMask          = 0x0400;
constexpr uint16_t kSpeedEqualToZeroMask       = 0x1000;
constexpr uint16_t kMaxSlippageReachedMask     = 0x2000;

Controller::Controller(Logger& log, uint8_t id)
  : log_(log),
    can_(Can::getInstance()),
    failure_(0)
{
  node_id_ = id << 1;
  sdo_receive = 0x600 + node_id_;
}

int32_t Controller::requestActualVelocity()
{
  return 0;
}

int16_t Controller::requestActualTorque()
{
  return 0;
}

void Controller::processNewData(utils::io::can::Frame& message)
{/*EMPTY*/}

int Controller::getFailure()
{
  return failure_;
}

}}  // namespace hyped::motor_control
