/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
 * Description:
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

#include "motor_control/motor.hpp"

#include <cstdint>
#include <iostream>

#include "motor_control/motor_controller.hpp"
#include "data/data.hpp"

namespace hyped {
namespace motor_control {

MotorController::MotorController(uint8_t id)
    : Thread(id)
{
  motor = new Motor();
  rpm = 0;
}

/**
  *  @brief  { Runs motor control thread. Switches to correct motor state
  *            based on state machine state }
  */
void MotorController::run()
{
  std::cout << "Starting motor controller" << std::endl;

  while (1) {
    nav = data.getNavigationData();
    state = data.getStateMachineData();
    switch (state.current_state) {
       case data::State::kIdle:
         this->setupMotors();
       case data::State::kAccelerating:
         this->accelerateMotors();
       case data::State::kDecelerating:
         this->decelerateMotors();
       case data::State::kEmergencyBraking:
         this->stopMotors();
       case data::State::kRunComplete:
         break;
       case data::State::kFailureStopped:
         break;
       case data::State::kExiting:
         break;
       case data::State::kFinished:
         break;
     }
  }
}

/**
  *  @brief  { Establish CAN connections with motor controllers }
  */
void MotorController::setupMotors()
{
  std::cout << "CAN connections established" << std::endl;
}

/**
  *  @brief  { Will accelerate motors until maximum acceleration distance is reached }
  */
void MotorController::accelerateMotors()
{
  while (state.current_state == data::State::kAccelerating) {
    if (state.critical_failure) {
      this->stopMotors();
    }
    state = data.getStateMachineData();
    nav = data.getNavigationData();
    rpm = calculateAccelerationRPM(nav.velocity);
    motor->setSpeed(rpm);
  }
}

/**
  *  @brief  { Will decelerate motors until total distance is reached }
  */
void MotorController::decelerateMotors()
{
  while (state.current_state == data::State::kDecelerating) {
    if (state.critical_failure) {
      this->stopMotors();
    }
    state = data.getStateMachineData();
    nav = data.getNavigationData();
    rpm = calculateDecelerationRPM(nav.velocity);
    motor->setSpeed(rpm);
  }
}

void MotorController::stopMotors()
{
  motor->setSpeed(0);
  std::cout << "Motors stopped" << std::endl;
}

/**
  *  @brief  { This function will run through slip ratio algorithm to calculate
  *            the desired acceleration RPM }
  *
  *  @param[in]  translational_velocity  { Value read from shared data structure }
  *
  *  @return  { Acceleration RPM calculation of type int }
  */
int32_t MotorController::calculateAccelerationRPM(uint32_t velocity)
{
  return rpm += 1000;  // dummy calculation to increase rpm
}

/**
  *  @brief  { This function will run through slip ratio algorithm to calculate
  *            the desired deceleration RPM }
  *
  *  @param[in]  translational_velocity  { Value read from shared data structure }
  *
  *  @return  { Deceleration RPM calculation of type int }
  */
int32_t MotorController::calculateDecelerationRPM(uint32_t velocity)
{
  return rpm -= 1000;  // dummy calculation to decrease rpm
}

}}  // namespace hyped::motor_control
