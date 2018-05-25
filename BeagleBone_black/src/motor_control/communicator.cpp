/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * Sends 'broadcast' CAN messages to all four controllers and requests data from individual
 * controllers.
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

#include "motor_control/communicator.hpp"
#include <cstdint>

#include "data/data.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/io/can.hpp"

using hyped::utils::io::can::Frame;

// TODO(anyone) set node IDs once we have some
constexpr uint8_t NODE_ALL  = 0x00;
// left-shifted 1 inorder to make it correct for the 11 bit CAN-ID
constexpr uint8_t NODE_1    = 0x01 << 1;
constexpr uint8_t NODE_2    = 0x02 << 1;
constexpr uint8_t NODE_3    = 0x03 << 1;
constexpr uint8_t NODE_4    = 0x04 << 1;

constexpr uint16_t OBD_RECEIVE = 0x600;


// Common CAN messages for all motor Controllers
// Device Control Command messages for all controllers
// pg49 CANOpen_Motion_Control.pdf
// TODO(anyone) look over again
constexpr Frame SHUTDOWN            = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x06,0,0}};
constexpr Frame SWITCH_ON_NO_ENABLE = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x07,0,0}};
constexpr Frame SWITCH_ON_ENABLE    = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x0F,0,0}};
constexpr Frame DISABLE_VOLTAGE     = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x00,0,0}};
constexpr Frame QUICK_STOP          = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x02,0,0}};
constexpr Frame DISABLE_OPERATION   = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x07,0,0}};
constexpr Frame ENABLE_OPERATION    = {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x00,0x0F,0,0}};
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
constexpr Frame SET_MAX_PROFILE_VELOCITY = {OBD_RECEIVE + NODE_ALL, false, 8, {0x23,0x60,0x7F,0,0,0,0x01,0}};

// Max motor speed is currently 0x00000100 = 256
// Assuming sub-index is 0x00 as not specified
constexpr Frame SET_MAX_MOTOR_SPEED = {OBD_RECEIVE + NODE_ALL, false, 8, {0x23,0x60,0x80,0,0,0,0x01,0}};


// profile velocity is currently 0x00000010 = 128
// Assuming sub-index is 0x00 as not specified
constexpr Frame SET_PROFILE_VELOCITY = {OBD_RECEIVE + NODE_ALL, false, 8, {0x23,0x60,0x81,0,0,0,0,0x10}};

// end velocity is zero because that is usually your speed when you have
// reached your target.
// Assuming sub-index is 0x00 as not specified 
// end velocity is currently 0x00000000 = 0
constexpr Frame SET_END_VELOCITY = {OBD_RECEIVE + NODE_ALL, false, 8, {0x23,0x60,0x81,0,0,0,0,0}};

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
constexpr Frame HALT_COMMAND =  {OBD_RECEIVE + NODE_ALL, false, 8, {0x2B,0x60,0x40,0,0x01,0x02,0,0}};

// Masks for profile velocity status
// pg123 'CANOpen_Motion_Control.pdf
constexpr uint16_t TARGET_REACHED_MASK              = 0x0400;
constexpr uint16_t SPEED_EQUAL_TO_ZERO_MASK         = 0x1000;
constexpr uint16_t MAX_SLIPPAGE_REACHED_MASK        = 0x2000;

namespace hyped {
namespace motor_control {

Communicator::Communicator(Logger& log)
  : log_(log),
    can_(Can::getInstance()),
    controller1_(log),
    controller2_(log),
    controller3_(log),
    controller4_(log)
{
  log_.INFO("MOTOR", "Controllers initialised");
}

/**
  *  @brief  { Register controllers to receive messages on CAN bus }
  */
void Communicator::registerControllers()
{}

/**
  *  @brief  { Send 'broadcast' CAN message containing target velocity to all four
  *            controllers by setting Node-ID = 0 }
  */
void Communicator::sendTargetVelocity(int32_t target_velocity)
{
  this->target_velocity_ = target_velocity;
}

/**
  *  @brief  { Send 'broadcast' CAN message containing target torque to all four
  *            controllers by setting Node-ID = 0 }
  */
void Communicator::sendTargetTorque(int16_t target_torque)
{
  this->target_torque_ = target_torque;
}

/**
  *  @brief  { Read actual velocity from each controller and return motor velocity struct }
  */
MotorVelocity Communicator::requestActualVelocity()
{
  motor_velocity_.motor_velocity_1 = controller1_.requestActualVelocity(target_velocity_);
  motor_velocity_.motor_velocity_2 = controller2_.requestActualVelocity(target_velocity_);
  motor_velocity_.motor_velocity_3 = controller3_.requestActualVelocity(target_velocity_);
  motor_velocity_.motor_velocity_4 = controller4_.requestActualVelocity(target_velocity_);

  log_.DBG2("MOTOR", "Actual Velocity: 1: %d, 2: %d, 3: %d, 4: %d"
    , motor_velocity_.motor_velocity_1
    , motor_velocity_.motor_velocity_2
    , motor_velocity_.motor_velocity_3
    , motor_velocity_.motor_velocity_4);

  return motor_velocity_;
}

/**
  *  @brief  { Read actual torque from each controller and return motor velocity struct }
  */
MotorTorque Communicator::requestActualTorque()
{
  motor_torque_.motor_torque_1 = controller1_.requestActualTorque();
  motor_torque_.motor_torque_2 = controller2_.requestActualTorque();
  motor_torque_.motor_torque_3 = controller3_.requestActualTorque();
  motor_torque_.motor_torque_4 = controller4_.requestActualTorque();

  log_.DBG2("MOTOR", "Actual Torque: 1: %d, 2: %d, 3: %d, 4: %d"
    , motor_torque_.motor_torque_1
    , motor_torque_.motor_torque_2
    , motor_torque_.motor_torque_3
    , motor_torque_.motor_torque_4);

  return motor_torque_;
}

/**
  *  @brief  { Read error address from each controller to check for error. Return false if no error }
  */
bool Communicator::checkStatus()
{
  return false;
}

}}  // namespace hyped::motor_control
