/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
 * Description:
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

#include "motor_control/main.hpp"

#include <cstdint>

#include "motor_control/motor.hpp"
#include "data/data.hpp"

namespace hyped {

using data::NavigationType;

namespace motor_control {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      motor_(log),
      rpm_(0),
      motorsSetUp_(false)
{}

/**
  *  @brief  { Runs motor control thread. Switches to correct motor state
  *            based on state machine state }
  */
void Main::run()
{
  log_.INFO("MOTOR", "Starting motor controller");

  while (1) {
    state_ = data_.getStateMachineData();
    switch (state_.current_state) {
       case data::State::kIdle:
         this->setupMotors();
         break;
       case data::State::kAccelerating:
         this->accelerateMotors();
         break;
       case data::State::kDecelerating:
         this->decelerateMotors();
         break;
       case data::State::kEmergencyBraking:
         this->stopMotors();
         break;
       case data::State::kRunComplete:
         goto exit_loop;
       case data::State::kFailureStopped:
         goto exit_loop;
       case data::State::kExiting:
         goto exit_loop;
       case data::State::kFinished:
         goto exit_loop;
     }
  }
  exit_loop: ;
}

/**
  *  @brief  { Establish CAN connections with motor controllers }
  */
void Main::setupMotors()
{
  if (!motorsSetUp_) {
    motor_data_ = { data::MotorState::kMotorIdle, 0, 0, 0, 0 };
    data_.setMotorData(motor_data_);
    motorsSetUp_ = true;
    log_.INFO("MOTOR", "Motor State: Idle");
  }
}

/**
  *  @brief  { Will accelerate motors until maximum acceleration distance is reached }
  */
void Main::accelerateMotors()
{
  while (state_.current_state == data::State::kAccelerating) {
    // Check for state machine critical failure flag
    state_ = data_.getStateMachineData();
    if (state_.critical_failure) {
      this->stopMotors();
      goto exit_loop;
    }

    // Check for motors critial failure flag
    motorFailure_ = motor_.checkStatus();
    if (motorFailure_) {
      log_.INFO("MOTOR", "Motor State: Motor Failure\n");
      MotorsRpm motors_rpm = motor_.getSpeed();
      motor_data_ = {
        data::MotorState::kCriticalFailure,
        motors_rpm.rpm_FL_,
        motors_rpm.rpm_FR_,
        motors_rpm.rpm_BL_,
        motors_rpm.rpm_BR_ };
      data_.setMotorData(motor_data_);
      this->stopMotors();
    }

    // Step up motor RPM
    log_.INFO("MOTOR", "Motor State: Accelerating\n");
    nav_ = data_.getNavigationData();
    rpm_ = calculateAccelerationRPM(nav_.velocity);
    motor_.setSpeed(rpm_);
    MotorsRpm motors_rpm = motor_.getSpeed();
    motor_data_ = {
      data::MotorState::kMotorAccelerating,
      motors_rpm.rpm_FL_,
      motors_rpm.rpm_FR_,
      motors_rpm.rpm_BL_,
      motors_rpm.rpm_BR_ };
    data_.setMotorData(motor_data_);
  }
  exit_loop: ;
}

/**
  *  @brief  { Will decelerate motors until total distance is reached }
  */
void Main::decelerateMotors()
{
  while (state_.current_state == data::State::kDecelerating) {
    // Check for state machine critical failure flag
    state_ = data_.getStateMachineData();
    if (state_.critical_failure) {
      this->stopMotors();
      goto exit_loop;
    }

    // Check for motors critical failure flag
    motorFailure_ = motor_.checkStatus();
    if (motorFailure_) {
      log_.INFO("MOTOR", "Motor State: Motor Failure\n");
      MotorsRpm motors_rpm = motor_.getSpeed();
      motor_data_ = {
        data::MotorState::kCriticalFailure,
        motors_rpm.rpm_FL_,
        motors_rpm.rpm_FR_,
        motors_rpm.rpm_BL_,
        motors_rpm.rpm_BR_ };
      data_.setMotorData(motor_data_);
      this->stopMotors();
    }

    // Step down motor RPM
    log_.INFO("MOTOR", "Motor State: Decelerating\n");
    nav_ = data_.getNavigationData();
    rpm_ = calculateDecelerationRPM(nav_.velocity);
    motor_.setSpeed(rpm_);
    MotorsRpm motors_rpm = motor_.getSpeed();
    // Updates the shared data on the motors RPM
    motor_data_ = {
      data::MotorState::kMotorDecelerating,
      motors_rpm.rpm_FL_,
      motors_rpm.rpm_FR_,
      motors_rpm.rpm_BL_,
      motors_rpm.rpm_BR_ };
    data_.setMotorData(motor_data_);
  }
  exit_loop: ;
}

void Main::stopMotors()
{
  motor_.setSpeed(0);
  bool allMotorsStopped = false;
  // Updates the shared data on the motors RPM while the motor is trying to stop
  while (!allMotorsStopped) {
    log_.DBG2("MOTOR", "Motor State: Stopping\n");
    MotorsRpm motors_rpm = motor_.getSpeed();
    data::Motors motor_data_ = {
      data::MotorState::kMotorStopping,
      motors_rpm.rpm_FL_,
      motors_rpm.rpm_FR_,
      motors_rpm.rpm_BL_,
      motors_rpm.rpm_BR_ };
    data_.setMotorData(motor_data_);
    if (motors_rpm.rpm_FL_ == 0 && motors_rpm.rpm_FR_ == 0 &&
      motors_rpm.rpm_BL_ == 0 && motors_rpm.rpm_BR_ == 0)
    {
      allMotorsStopped = true;
    }
  }
  MotorsRpm motors_rpm = motor_.getSpeed();
  data::Motors motor_data_ = {
    data::MotorState::kMotorStopped,
    motors_rpm.rpm_FL_,
    motors_rpm.rpm_FR_,
    motors_rpm.rpm_BL_,
    motors_rpm.rpm_BR_ };
  data_.setMotorData(motor_data_);
  log_.INFO("MOTOR", "Motor State: Stopped\n");
}

/**
  *  @brief  { This function will run through slip ratio algorithm to calculate
  *            the desired acceleration RPM }
  *
  *  @param[in]  translational_velocity  { Value read from shared data structure }
  *
  *  @return  { Acceleration RPM calculation of type int }
  */
int32_t Main::calculateAccelerationRPM(NavigationType velocity)
{
  return rpm_ += 100;  // dummy calculation to increase rpm
}

/**
  *  @brief  { This function will run through slip ratio algorithm to calculate
  *            the desired deceleration RPM }
  *
  *  @param[in]  translational_velocity  { Value read from shared data structure }
  *
  *  @return  { Deceleration RPM calculation of type int }
  */
int32_t Main::calculateDecelerationRPM(NavigationType velocity)
{
  return rpm_ -= 100;  // dummy calculation to decrease rpm
}

}}  // namespace hyped::motor_control
