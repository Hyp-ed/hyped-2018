/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * Abstracts the four controller objects away from the Motor Control Main, updates the data structure
 * and relays data to Main accordingly.
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
  : data_(data::Data::getInstance()),
    log_(log),
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
  log_.INFO("MOTOR", "Controllers registered on CAN bus");
}

void Communicator::configureControllers()
{
  controller1_.configure();
  controller2_.configure();
  controller3_.configure();
  controller4_.configure();
  if (controller1_.getConfiguartionStatus() == true
      && controller2_.getConfiguartionStatus() == true
      && controller3_.getConfiguartionStatus() == true
      && controller1_.getConfiguartionStatus() == true)
      {
        log_.INFO("MOTOR", "Motors are configured for launch");
      } else {
        data::Motors motor_data_ = { data::MotorState::kConfigurationError,
                                     0, 0, 0, 0, 0, 0, 0, 0 };
        data_.setMotorData(motor_data_);
      }
}

bool Communicator::enterOperational()
{
  controller1_.enterOperational();
  controller2_.enterOperational();
  controller3_.enterOperational();
  controller4_.enterOperational();
  if (controller1_.getControllerState() == kOperationEnabled
      && controller1_.getControllerState() == kOperationEnabled
      && controller1_.getControllerState() == kOperationEnabled
      && controller1_.getControllerState() == kOperationEnabled)
      {
        return true;
      }
  return false;
}

void Communicator::enterPreOperational()
{
  controller1_.enterPreOperational();
  controller2_.enterPreOperational();
  controller3_.enterPreOperational();
  controller4_.enterPreOperational();
}

void Communicator::sendTargetVelocity(int32_t target_velocity)
{
  controller1_.sendTargetVelocity(target_velocity);
  controller2_.sendTargetVelocity(target_velocity);
  controller3_.sendTargetVelocity(target_velocity);
  controller4_.sendTargetVelocity(target_velocity);
}

void Communicator::sendTargetTorque(int16_t target_torque)
{
  controller1_.sendTargetTorque(target_torque);
  controller2_.sendTargetTorque(target_torque);
  controller3_.sendTargetTorque(target_torque);
  controller4_.sendTargetTorque(target_torque);
}

MotorVelocity Communicator::requestActualVelocity()
{
  controller1_.updateActualVelocity();
  controller2_.updateActualVelocity();
  controller3_.updateActualVelocity();
  controller4_.updateActualVelocity();
  motor_velocity_.velocity_1 = controller1_.getVelocity();
  motor_velocity_.velocity_2 = controller2_.getVelocity();
  motor_velocity_.velocity_3 = controller3_.getVelocity();
  motor_velocity_.velocity_4 = controller4_.getVelocity();

  log_.DBG2("MOTOR", "Actual Velocity: 1: %d, 2: %d, 3: %d, 4: %d"
    , motor_velocity_.velocity_1
    , motor_velocity_.velocity_2
    , motor_velocity_.velocity_3
    , motor_velocity_.velocity_4);

  return motor_velocity_;
}

MotorTorque Communicator::requestActualTorque()
{
  controller1_.updateActualTorque();
  controller2_.updateActualTorque();
  controller3_.updateActualTorque();
  controller4_.updateActualTorque();
  motor_torque_.torque_1 = controller1_.getTorque();
  motor_torque_.torque_2 = controller2_.getTorque();
  motor_torque_.torque_3 = controller3_.getTorque();
  motor_torque_.torque_4 = controller4_.getTorque();

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
