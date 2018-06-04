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

#include "motor_control/communicator.hpp"
#include "data/data.hpp"

namespace hyped {

using data::NavigationType;

namespace motor_control {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      communicator_(log),
      target_velocity_(0),
      target_torque_(0),
      motors_set_up_(false),
      motors_operational_(false),
      run_(true),
      all_motors_stopped_(false)
{}

void Main::run()
{
  log_.INFO("MOTOR", "Starting motor controller");
  while (run_) {
    state_ = data_.getStateMachineData();
    if (state_.current_state == data::State::kIdle) {
      this->setupMotors();
    } else if (state_.current_state == data::State::kAccelerating) {
      this->accelerateMotors();
    } else if (state_.current_state == data::State::kDecelerating) {
      this->decelerateMotors();
    } else if (state_.current_state == data::State::kFailureStopped) {
      communicator_.enterPreOperational();
    } else if (state_.current_state == data::State::kEmergencyBraking) {
      this->stopMotors();
    } else {
      run_ = false;
    }
  }
}

void Main::setupMotors()
{
  if (!motors_set_up_) {
    communicator_.registerControllers();
    communicator_.configureControllers();
    data::Motors motor_data_ = data_.getMotorData();
    motor_data_ = { data::MotorState::kPreOperational, 0, 0, 0, 0, 0, 0, 0, 0 };
    data_.setMotorData(motor_data_);
    motors_set_up_ = true;
    log_.INFO("MOTOR", "Motor State: Idle");
  }
}

void Main::accelerateMotors()
{
  if (!motors_operational_) {
    bool operational = communicator_.enterOperational();
    if (operational) {
      motors_operational_ = true;
    } else {
      log_.ERR("MOTOR", "Controllers not operational");
      data::Motors motor_data_ = { data::MotorState::kCriticalFailure, 0, 0, 0, 0, 0, 0, 0, 0 };
      data_.setMotorData(motor_data_);
      return;
    }
  }
  log_.INFO("MOTOR", "Motor State: Accelerating\n");
  while (state_.current_state == data::State::kAccelerating) {
    // Check for state machine critical failure flag
    state_ = data_.getStateMachineData();
    if (state_.critical_failure) {
      this->stopMotors();
      break;
    }

    // Check for motors critial failure flag
    motor_failure_ = communicator_.checkFailure();
    if (motor_failure_) {
      log_.INFO("MOTOR", "Motor State: Motor Failure\n");
      MotorVelocity motor_velocity = communicator_.requestActualVelocity();
      MotorTorque motor_torque     = communicator_.requestActualTorque();
      // Write critical failure flag and motor data to data structure
      data::Motors motor_data_ = {
          data::MotorState::kCriticalFailure,
          motor_velocity.velocity_1,
          motor_velocity.velocity_2,
          motor_velocity.velocity_3,
          motor_velocity.velocity_4,
          motor_torque.torque_1,
          motor_torque.torque_2,
          motor_torque.torque_3,
          motor_torque.torque_4 };
      data_.setMotorData(motor_data_);
      this->stopMotors();
      break;
    }

    // Step up motor velocity
    log_.DBG2("MOTOR", "Motor State: Accelerating\n");
    data::Navigation nav_ = data_.getNavigationData();
    target_velocity_      = accelerationVelocity(nav_.velocity);
    target_torque_        = accelerationTorque(nav_.velocity);
    communicator_.sendTargetVelocity(target_velocity_);
    communicator_.sendTargetTorque(target_torque_);
    MotorVelocity motor_velocity = communicator_.requestActualVelocity();
    MotorTorque motor_torque     = communicator_.requestActualTorque();
    // Write current state (accelerating) and motor data to data structure
    data::Motors motor_data_ = {
        data::MotorState::kMotorAccelerating,
        motor_velocity.velocity_1,
        motor_velocity.velocity_2,
        motor_velocity.velocity_3,
        motor_velocity.velocity_4,
        motor_torque.torque_1,
        motor_torque.torque_2,
        motor_torque.torque_3,
        motor_torque.torque_4 };
    data_.setMotorData(motor_data_);
  }
}

void Main::decelerateMotors()
{
  log_.INFO("MOTOR", "Motor State: Deccelerating\n");
  while (state_.current_state == data::State::kDecelerating) {
    // Check for state machine critical failure flag
    state_ = data_.getStateMachineData();
    if (state_.critical_failure) {
      this->stopMotors();
      break;
    }

    // Check for motors critical failure flag
    motor_failure_ = communicator_.checkFailure();
    if (motor_failure_) {
      log_.INFO("MOTOR", "Motor State: Motor Failure\n");
      MotorVelocity motor_velocity = communicator_.requestActualVelocity();
      MotorTorque motor_torque     = communicator_.requestActualTorque();
      // Write critical failure flag and motor data to data structure
      data::Motors motor_data_ = {
          data::MotorState::kCriticalFailure,
          motor_velocity.velocity_1,
          motor_velocity.velocity_2,
          motor_velocity.velocity_3,
          motor_velocity.velocity_4,
          motor_torque.torque_1,
          motor_torque.torque_2,
          motor_torque.torque_3,
          motor_torque.torque_4 };
      data_.setMotorData(motor_data_);
      this->stopMotors();
      break;
    }

    // Step down motor RPM
    log_.DBG2("MOTOR", "Motor State: Deccelerating\n");
    data::Navigation nav_ = data_.getNavigationData();
    target_velocity_      = decelerationVelocity(nav_.velocity);
    target_torque_        = decelerationTorque(nav_.velocity);
    communicator_.sendTargetVelocity(target_velocity_);
    communicator_.sendTargetTorque(target_torque_);
    MotorVelocity motor_velocity = communicator_.requestActualVelocity();
    MotorTorque motor_torque     = communicator_.requestActualTorque();
    // Write current state (decelerating) and motor data to data structure
    data::Motors motor_data_ = {
        data::MotorState::kMotorDecelerating,
        motor_velocity.velocity_1,
        motor_velocity.velocity_2,
        motor_velocity.velocity_3,
        motor_velocity.velocity_4,
        motor_torque.torque_1,
        motor_torque.torque_2,
        motor_torque.torque_3,
        motor_torque.torque_4 };
    data_.setMotorData(motor_data_);
  }
}

void Main::stopMotors()
{
  communicator_.quickStopAll();
  // Updates the motor data while motors are stopping
  while (!all_motors_stopped_) {
    log_.DBG2("MOTOR", "Motor State: Stopping\n");
    MotorVelocity motor_velocity = communicator_.requestActualVelocity();
    MotorTorque motor_torque     = communicator_.requestActualTorque();
    // Write current state (stopping) and motor data to data structure
    data::Motors motor_data_ = {
        data::MotorState::kMotorStopping,
        motor_velocity.velocity_1,
        motor_velocity.velocity_2,
        motor_velocity.velocity_3,
        motor_velocity.velocity_4,
        motor_torque.torque_1,
        motor_torque.torque_2,
        motor_torque.torque_3,
        motor_torque.torque_4 };
    data_.setMotorData(motor_data_);

    if (motor_velocity.velocity_1 == 0 && motor_velocity.velocity_2 == 0 &&
        motor_velocity.velocity_3 == 0 && motor_velocity.velocity_4 == 0)
    {
      all_motors_stopped_ = true;
      log_.INFO("MOTOR", "Motor State: Stopped\n");
    }
  }

  MotorVelocity motor_velocity = communicator_.requestActualVelocity();
  MotorTorque motor_torque     = communicator_.requestActualTorque();
  // Write current state (stopped) and motor data to data structure
  data::Motors motor_data_ = {
      data::MotorState::kMotorStopped,
      motor_velocity.velocity_1,
      motor_velocity.velocity_2,
      motor_velocity.velocity_3,
      motor_velocity.velocity_4,
      motor_torque.torque_1,
      motor_torque.torque_2,
      motor_torque.torque_3,
      motor_torque.torque_4 };
  data_.setMotorData(motor_data_);
  log_.DBG2("MOTOR", "Motor State: Stopped\n");
}

int32_t Main::accelerationVelocity(NavigationType velocity)
{
  return target_velocity_ += 100;  // dummy calculation to increase rpm
}

int32_t Main::decelerationVelocity(NavigationType velocity)
{
  return target_velocity_ -= 100;  // dummy calculation to decrease rpm
}

int16_t Main::accelerationTorque(NavigationType velocity)
{
  return 0;
}

int16_t Main::decelerationTorque(NavigationType velocity)
{
  return 0;
}

}}  // namespace hyped::motor_control
