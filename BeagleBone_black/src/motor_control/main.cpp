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

#include "motor_control/main.hpp"

#include <cstdint>
#include <iostream>

#include "motor_control/motor.hpp"

#include "data/data.hpp"

namespace hyped {
namespace motor_control {

Main::Main(uint8_t id)
    : Thread(id)
{
  motor = new Motor();
  rpm = 0;
}

/**
  *  @brief  { Runs motor control thread. Switches to correct motor state
  *            based on state machine state }
  */
void Main::run()
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
void Main::setupMotors()
{
  data::Motors motor_data = { data::MotorState::kMotorIdle, 0, 0, 0, 0 };
  data.setMotorData(motor_data);
  std::cout << "CAN connections established" << std::endl;
}

/**
  *  @brief  { Will accelerate motors until maximum acceleration distance is reached }
  */
void Main::accelerateMotors()
{
  while (state.current_state == data::State::kAccelerating) {
    if (state.critical_failure) {
      this->stopMotors();
    }
    state = data.getStateMachineData();
    nav = data.getNavigationData();
    rpm = calculateAccelerationRPM(nav.velocity);
    motor->setSpeed(rpm);
    MotorsRpm motors_rpm = motor->getSpeed();
    // Updates the shared data on the motors RPM
    data::Motors motor_data = { data::MotorState::kMotorAccelerating,
    motors_rpm.rpm_FL, motors_rpm.rpm_FR, motors_rpm.rpm_BL, motors_rpm.rpm_BR };
    data.setMotorData(motor_data);
  }
}

/**
  *  @brief  { Will decelerate motors until total distance is reached }
  */
void Main::decelerateMotors()
{
  while (state.current_state == data::State::kDecelerating) {
    if (state.critical_failure) {
      this->stopMotors();
    }
    state = data.getStateMachineData();
    nav = data.getNavigationData();
    rpm = calculateDecelerationRPM(nav.velocity);
    motor->setSpeed(rpm);
    MotorsRpm motors_rpm = motor->getSpeed();
    // Updates the shared data on the motors RPM
    data::Motors motor_data = { data::MotorState::kMotorDecelerating,
    motors_rpm.rpm_FL, motors_rpm.rpm_FR, motors_rpm.rpm_BL, motors_rpm.rpm_BR };
    data.setMotorData(motor_data);
  }
}

void Main::stopMotors()
{
  motor->setSpeed(0);
  bool isAllStop = false;
  // Updates the shared data on the motors RPM while the motor is trying to stop
  while (!isAllStop) {
    std::cout << "Decelerating" << std::endl;
    MotorsRpm motors_rpm = motor->getSpeed();
    data::Motors motor_data = { data::MotorState::kMotorDecelerating,
    motors_rpm.rpm_FL, motors_rpm.rpm_FR, motors_rpm.rpm_BL, motors_rpm.rpm_BR };
    data.setMotorData(motor_data);
    if (motors_rpm.rpm_FL == 0 && motors_rpm.rpm_FR == 0 &&
      motors_rpm.rpm_BL == 0 && motors_rpm.rpm_BR == 0)
    {
      isAllStop = true;
    }
  }
  // The motor has stopped update the data structure
  MotorsRpm motors_rpm = motor->getSpeed();
  data::Motors motor_data = { data::MotorState::kMotorStopped,
  motors_rpm.rpm_FL, motors_rpm.rpm_FR, motors_rpm.rpm_BL, motors_rpm.rpm_BR };
  data.setMotorData(motor_data);
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
int32_t Main::calculateAccelerationRPM(uint32_t velocity)
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
int32_t Main::calculateDecelerationRPM(uint32_t velocity)
{
  return rpm -= 1000;  // dummy calculation to decrease rpm
}

}}  // namespace hyped::motor_control
