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
