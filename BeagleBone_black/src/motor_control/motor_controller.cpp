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

void MotorController::run()
{
  std::cout << "Starting motor controller" << std::endl;

  // while (1) {
  //   data::Data& data = data:: Data::getInstance();
  //   nav = data.getStateData();
  //   state = data.getStateData();
  //   switch (state.current_state) {
  //      case Idle:
  //        this->setUpMotors();
  //      case Accelerating:
  //        this->accelerateMotors();
  //      case Decelerating:
  //        this->decelerateMotors();
  //      case EmergencyBraking:
  //        this->stopMotors();
  //      case RunComplete:
  //        break;
  //      case FailureStopped:
  //        break;
  //      case Exiting:
  //        break;
  //      case Finished:
  //        break;
  //    }
  // }
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
  // while (1) {
  //   // Read translational velocity from shared data structure
  //   nav = data.getNavigationData();
  //   rpm = calculateAccelerationRPM(nav.velocity);
  //   motor->setSpeed(rpm);
  // }
}

/**
  *  @brief  { Will decelerate motors until total distance is reached }
  */
void MotorController::decelerateMotors()
{
  // while (1) {
  //   // Read translational velocity from shared data structure
  //   nav = data.getNavigationData();
  //   rpm = calculateDecelerationRPM(nav.velocity);
  //   motor->setSpeed(rpm);
  // }
}

void MotorController::stopMotors()
{
  // motor->setSpeed(0);
  // std::cout << "Motors stopped" << std::endl;
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
