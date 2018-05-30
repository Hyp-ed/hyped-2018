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

namespace hyped {
namespace motor_control {

Communicator::Communicator(Logger& log)
  : log_(log),
    controller1_(log, 1),
    controller2_(log, 2),
    controller3_(log, 3),
    controller4_(log, 4)
{
  log_.INFO("MOTOR", "Controllers initialised\n");
}

void Communicator::registerControllers()
{
  controller1_.registerController();
  controller2_.registerController();
  controller3_.registerController();
  controller4_.registerController();
}

void Communicator::configureControllers()
{
  controller1_.configure();
  controller2_.configure();
  controller3_.configure();
  controller4_.configure();
}

void Communicator::sendTargetVelocity(int32_t target_velocity)
{
  controller1_.sendTargetVelocity(target_velocity);
  controller2_.sendTargetVelocity(target_velocity);
  controller3_.sendTargetVelocity(target_velocity);
  controller4_.sendTargetVelocity(target_velocity);
  this->target_velocity_ = target_velocity;
}

void Communicator::sendTargetTorque(int16_t target_torque)
{
  controller1_.sendTargetTorque(target_torque);
  controller2_.sendTargetTorque(target_torque);
  controller3_.sendTargetTorque(target_torque);
  controller4_.sendTargetTorque(target_torque);
  this->target_torque_ = target_torque;
}

MotorVelocity Communicator::requestActualVelocity()
{
  motor_velocity_.velocity_1 = controller1_.requestActualVelocity();
  motor_velocity_.velocity_2 = controller2_.requestActualVelocity();
  motor_velocity_.velocity_3 = controller3_.requestActualVelocity();
  motor_velocity_.velocity_4 = controller4_.requestActualVelocity();

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

void Communicator::quickStopAll()
{
  controller1_.quickStop();
  controller2_.quickStop();
  controller3_.quickStop();
  controller4_.quickStop();
}

bool Communicator::checkFailure()
{
  bool f1, f2, f3, f4;
  f1 = controller1_.getFailure();
  f2 = controller2_.getFailure();
  f3 = controller3_.getFailure();
  f4 = controller4_.getFailure();
  return f1 || f2 || f3 || f4;
}

}}  // namespace hyped::motor_control
