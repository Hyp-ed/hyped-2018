/*
 * Author: Sean Mullan
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

using hyped::utils::io::can::Frame

// Common CAN messages for all motor Controllers

// struct Frame {
//   static constexpr uint32_t kExtendedMask = 0x80000000U;
//   uint32_t  id;
//   bool      extended;
//   uint8_t   len;
//   uint8_t   data[8];
// };

// Device Control Command messages for all controllers
// pg49 CANOpen_Motion_Control.pdf
constexpr Frame SHUTDOWN            = {0, false, 2, {0,6,0,0,0,0,0,0}};
constexpr Frame SWITCH_ON_NO_ENABLE = {0, false, 2, {0,7,0,0,0,0,0,0}};
constexpr Frame SWITCH_ON_ENABLE    = {0, false, 2, {0,15,0,0,0,0,0,0}};
constexpr Frame DISABLE_VOLTAGE     = {0, false, 2, {0,0,0,0,0,0,0,0}};
constexpr Frame QUICK_STOP          = {0, false, 2, {0,2,0,0,0,0,0,0}};
constexpr Frame DISABLE_OPERATION   = {0, false, 2, {0,7,0,0,0,0,0,0}};
constexpr Frame ENABLE_OPERATION    = {0, false, 2, {0,15,0,0,0,0,0,0}};

// Status masks, used to mask the return result of a CAN message
// pg51 CANOpen_Motion_Control.pdf
constexpr uint16_t NOT_READY_TO_SWITCH_ON_MASK    = 0x0000;
constexpr uint16_t SWITCH_ON_DISABLED_MASK        = 0x0040;
constexpr uint16_t READY_TO_SWITCH_ON_MASK        = 0x0021;
constexpr uint16_t SWITCHED_ON_MASK               = 0x0023;
constexpr uint16_t OPERATION_ENABLED_MASK         = 0x0027;
constexpr uint16_t QUICK_STOP_ACTIVE_MASK         = 0x0007;
constexpr uint16_t FAULT_REACTION_ACTIVE_MASK     = 0x0008;
constexpr uint16_t FAULT_MASK                     = 0x0008;


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
  log_.INFO("MOTOR", "Controllers initialised\n");
}

void Communicator::registerControllers()
{}

void Communicator::sendTargetVelocity(int32_t target_velocity)
{
  this->target_velocity_ = target_velocity;
}

void Communicator::sendTargetTorque(int16_t target_torque)
{
  this->target_torque_ = target_torque;
}

MotorVelocity Communicator::requestActualVelocity()
{
  motor_velocity_.velocity_1 = controller1_.requestActualVelocity(target_velocity_);
  motor_velocity_.velocity_2 = controller2_.requestActualVelocity(target_velocity_);
  motor_velocity_.velocity_3 = controller3_.requestActualVelocity(target_velocity_);
  motor_velocity_.velocity_4 = controller4_.requestActualVelocity(target_velocity_);

  log_.DBG2("MOTOR", "Actual Velocity: 1: %d, 2: %d, 3: %d, 4: %d"
    , motor_velocity_.velocity_1
    , motor_velocity_.velocity_2
    , motor_velocity_.velocity_3
    , motor_velocity_.velocity_4);

  return motor_velocity_;
}

MotorTorque Communicator::requestActualTorque()
{
  motor_torque_.torque_1 = controller1_.requestActualTorque();
  motor_torque_.torque_2 = controller2_.requestActualTorque();
  motor_torque_.torque_3 = controller3_.requestActualTorque();
  motor_torque_.torque_4 = controller4_.requestActualTorque();

  log_.DBG2("MOTOR", "Actual Torque: 1: %d, 2: %d, 3: %d, 4: %d"
    , motor_torque_.torque_1
    , motor_torque_.torque_2
    , motor_torque_.torque_3
    , motor_torque_.torque_4);

  return motor_torque_;
}

bool Communicator::checkFailure()
{
  return false;
}

}}  // namespace hyped::motor_control
