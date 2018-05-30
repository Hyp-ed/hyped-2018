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

/* Can Frames composed as follows:
 * CAN ID:    COB-ID = Function + Node ID,
 * Len   :    Size of object dictionary entry
 * Byte 0:    Command
 * Bytes 1-2: Object Dictionary Index
 * Byte  3:   Object Dictionary Sub-Index
 * Bytes 4-7: Data
 */

 /* Commands:
  * 0x40: Read dictionary object
  * 0x23: Write 4 bytes
  * 0x27: Write 3 bytes
  * 0x2B: Write 2 bytes
  * 0x2F: Write 2 Bytes
  */

constexpr uint16_t sdo_receive = 0x600;

// Status masks, used to mask the return result of a CAN message
// pg51 CANOpen_Motion_Control.pdf
constexpr uint16_t kNotReadyToSwitch       = 0x0000;
constexpr uint16_t kSwitchOnDisabled       = 0x0040;
constexpr uint16_t kReadyToSwitchOn        = 0x0021;
constexpr uint16_t kSwitchedOn             = 0x0023;
constexpr uint16_t kOperationEnabled       = 0x0027;
constexpr uint16_t kQuickStopActive        = 0x0007;
constexpr uint16_t kFaultReactionActive    = 0x0008;
// Unsure about this check datasheet TODO(anyone)
constexpr uint16_t kFault                  = 0x0008;

// Masks for profile velocity status
// pg123 'CANOpen_Motion_Control.pdf
constexpr uint16_t kTargetReached          = 0x0400;
constexpr uint16_t kSpeedEqualToZero       = 0x1000;
constexpr uint16_t kMaxSlippageReached     = 0x2000;

Controller::Controller(Logger& log, uint8_t id)
  : log_(log),
    can_(Can::getInstance()),
    failure_(false)
{
  node_id_ = id << 1;
  sdo_receive = 0x600 + node_id_;
  init();
}

void Controller::registerController()
{/*EMPTY*/}

void Controller::init()
{/*EMPTY*/}

void Controller::sendTargetVelocity(int32_t target_velocity)
{/*EMPTY*/}

void Controller::sendTargetTorque(int16_t target_torque)
{/*EMPTY*/}

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

void Controller::quickStop()
{/*EMPTY*/}

bool Controller::getFailure()
{
  return failure_;
}

uint8_t Controller::getId()
{
  return node_id_;
}

}}  // namespace hyped::motor_control
