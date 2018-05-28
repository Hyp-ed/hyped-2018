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

// Common CAN messages for all motor Controllers
// Device Control Command messages for all controllers
// pg49 CANOpen_Motion_Control.pdf
// TODO(anyone) look over again
constexpr Frame SHUTDOWN            = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x06, 0, 0}}; //NOLINT
constexpr Frame SWITCH_ON_NO_ENABLE = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x07, 0, 0}}; //NOLINT
constexpr Frame SWITCH_ON_ENABLE    = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x0F, 0, 0}}; //NOLINT
constexpr Frame DISABLE_VOLTAGE     = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x00, 0, 0}}; //NOLINT
constexpr Frame QUICK_STOP          = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x02, 0, 0}}; //NOLINT
constexpr Frame DISABLE_OPERATION   = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x07, 0, 0}}; //NOLINT
constexpr Frame ENABLE_OPERATION    = {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x00, 0x0F, 0, 0}}; //NOLINT
// TODO(anyone) Add Fault Reset but unsure on how to do it

// Status masks, used to mask the return result of a CAN message
// pg51 CANOpen_Motion_Control.pdf
constexpr uint16_t NOT_READY_TO_SWITCH_ON_MASK    = 0x0000;
constexpr uint16_t SWITCH_ON_DISABLED_MASK        = 0x0040;
constexpr uint16_t READY_TO_SWITCH_ON_MASK        = 0x0021;
constexpr uint16_t SWITCHED_ON_MASK               = 0x0023;
constexpr uint16_t OPERATION_ENABLED_MASK         = 0x0027;
constexpr uint16_t QUICK_STOP_ACTIVE_MASK         = 0x0007;
constexpr uint16_t FAULT_REACTION_ACTIVE_MASK     = 0x0008;
// Unsure about this check datasheet TODO(anyone)
constexpr uint16_t FAULT_MASK                     = 0x0008;

// All Defined in chapter 12 of CANOpen_Motion_Control.pdf

// Max profile speed is currently 0x00000100 = 256
// Assuming sub-index is 0x00 as not specified
constexpr Frame SET_MAX_PROFILE_VELOCITY = {sdo_receive, false, 8, {0x23, 0x60, 0x7F, 0, 0, 0, 0x01, 0}}; //NOLINT

// Max motor speed is currently 0x00000100 = 256
// Assuming sub-index is 0x00 as not specified
constexpr Frame SET_MAX_MOTOR_SPEED = {sdo_receive, false, 8, {0x23, 0x60, 0x80, 0, 0, 0, 0x01, 0}}; //NOLINT


// profile velocity is currently 0x00000010 = 128
// Assuming sub-index is 0x00 as not specified
constexpr Frame SET_PROFILE_VELOCITY = {sdo_receive, false, 8, {0x23, 0x60, 0x81, 0, 0, 0, 0, 0x10}}; //NOLINT

// end velocity is zero because that is usually your speed when you have
// reached your target.
// Assuming sub-index is 0x00 as not specified
// end velocity is currently 0x00000000 = 0
constexpr Frame SET_END_VELOCITY = {sdo_receive, false, 8, {0x23, 0x60, 0x81, 0, 0, 0, 0, 0}}; //NOLINT

/* TODO(anyone) profile acceleration
 See chapter 12.3.8 defined using acceleration units (needs to be defined) */
// TODO(anyone) profile deceleration. Same as profile acceleration
// TODO(anyone) quick stop deceleration. See 12.3.10
// TODO(anyone) motion profile type. See 12.3.11 (needs to be discussed)
// TODO(anyone) max acceleration. See 12.3.12 (needs to be discussed)
// TODO(anyone) max deceleration. See 12.3.13 (needs to be discussed)

// See pg 120 for profile velocity mode
// Gives Halt command to all motors from velocity mode (QUICK_STOP)
// TODO(anyone) look over again
constexpr Frame HALT_COMMAND =  {sdo_receive, false, 8, {0x2B, 0x60, 0x40, 0, 0x01, 0x02, 0, 0}}; //NOLINT

// Masks for profile velocity status
// pg123 'CANOpen_Motion_Control.pdf
constexpr uint16_t TARGET_REACHED_MASK              = 0x0400;
constexpr uint16_t SPEED_EQUAL_TO_ZERO_MASK         = 0x1000;
constexpr uint16_t MAX_SLIPPAGE_REACHED_MASK        = 0x2000;

Controller::Controller(Logger& log, uint8_t id)
  : log_(log),
    can_(Can::getInstance())
{
  node_id_ = id << 1;
  sdo_receive = 0x600 + node_id_;
}

int32_t Controller::requestActualVelocity(int32_t target_velocity)
{
  return target_velocity;
}

int16_t Controller::requestActualTorque()
{
  return 0;
}

}}  // namespace hyped::motor_control
