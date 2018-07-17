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

namespace hyped {

using utils::System;

namespace motor_control {

Communicator::Communicator(Logger& log)
  : sys_(System::getSystem()),
    data_(data::Data::getInstance()),
    log_(log),
    critical_failure_(false)
{
  if (!sys_.fake_motors) {
    controller1_ = new Controller(log, 1);
    controller2_ = new Controller(log, 2);
    controller3_ = new Controller(log, 3);
    controller4_ = new Controller(log, 4);
  } else {
    if (sys_.fail_motors) {
      controller1_ = new FakeController(log, 1, true);
    } else {
      controller1_ = new FakeController(log, 1, false);
    }
    controller2_ = new FakeController(log, 2, false);
    controller3_ = new FakeController(log, 3, false);
    controller4_ = new FakeController(log, 4, false);

    log_.INFO("MOTOR", "Fake motors created");
  }
}

void Communicator::registerControllers()
{
  controller1_->registerController();
  controller2_->registerController();
  controller3_->registerController();
  controller4_->registerController();
  log_.INFO("MOTOR", "Controllers registered on CAN bus");
}

void Communicator::configureControllers()
{
  controller1_->configure();
  controller2_->configure();
  controller3_->configure();
  controller4_->configure();
  bool f1, f2, f3, f4;
  f1 = controller1_->getFailure();
  f2 = controller2_->getFailure();
  f3 = controller3_->getFailure();
  f4 = controller4_->getFailure();
  if (f1 || f2 || f3 || f4) {
    critical_failure_ = true;
    log_.ERR("MOTOR", "COMMUNICATION FAILURE");
  } else {
    log_.INFO("MOTOR", "All motors are configured");
  }
}

void Communicator::prepareMotors()
{
  controller1_->enterOperational();
  controller2_->enterOperational();
  controller3_->enterOperational();
  controller4_->enterOperational();
  if (controller1_->getControllerState() != kOperationEnabled
     || controller1_->getControllerState() != kOperationEnabled
     || controller1_->getControllerState() != kOperationEnabled
     || controller1_->getControllerState() != kOperationEnabled)
  {
    critical_failure_ = true;
    log_.ERR("MOTOR", "Motors not operational");
  } else {
    log_.INFO("MOTOR", "Motors are ready for launch");
  }
}

void Communicator::enterPreOperational()
{
  controller1_->enterPreOperational();
  controller2_->enterPreOperational();
  controller3_->enterPreOperational();
  controller4_->enterPreOperational();
}

void Communicator::sendTargetVelocity(int32_t target_velocity)
{
  // TODO(anyone) need to check if this is correct for our set-up of motors
  controller1_->sendTargetVelocity(target_velocity);
  controller2_->sendTargetVelocity(-target_velocity);
  controller3_->sendTargetVelocity(target_velocity);
  controller4_->sendTargetVelocity(-target_velocity);
}

MotorVelocity Communicator::requestActualVelocity()
{
  controller1_->updateActualVelocity();
  controller2_->updateActualVelocity();
  controller3_->updateActualVelocity();
  controller4_->updateActualVelocity();
  motor_velocity_.velocity_1 = controller1_->getVelocity();
  motor_velocity_.velocity_2 = -controller2_->getVelocity();
  motor_velocity_.velocity_3 = controller3_->getVelocity();
  motor_velocity_.velocity_4 = -controller4_->getVelocity();

  log_.DBG2("MOTOR", "Actual Velocity: 1: %d, 2: %d, 3: %d, 4: %d"
    , motor_velocity_.velocity_1
    , motor_velocity_.velocity_2
    , motor_velocity_.velocity_3
    , motor_velocity_.velocity_4);

  return motor_velocity_;
}

void Communicator::quickStopAll()
{
  controller1_->quickStop();
  controller2_->quickStop();
  controller3_->quickStop();
  controller4_->quickStop();
}

void Communicator::healthCheck()
{
  controller1_->healthCheck();
  controller2_->healthCheck();
  controller3_->healthCheck();
  controller4_->healthCheck();
  bool f1, f2, f3, f4;
  f1 = controller1_->getFailure();
  f2 = controller2_->getFailure();
  f3 = controller3_->getFailure();
  f4 = controller4_->getFailure();
  if (f1 || f2 || f3 || f4) {
    critical_failure_ = true;
  }
}

bool Communicator::getFailure()
{
  return critical_failure_;
}

}}  // namespace hyped::motor_control
