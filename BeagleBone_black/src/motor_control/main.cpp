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

#include <math.h>
#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif
#include <cstdint>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#include "motor_control/communicator.hpp"
#include "data/data.hpp"
#include "utils/system.hpp"

namespace hyped {

using data::NavigationType;
using utils::System;
using utils::Timer;

namespace motor_control {

constexpr double  kHalbachRadius    = 0.148;
const std::string kAccelerationData = "../BeagleBone_black/data/configuration/AccelerationSlip.txt";
const std::string kDecelerationData = "../BeagleBone_black/data/configuration/DecelerationSlip.txt";

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      post_calibration_barrier_(System::getSystem().navigation_motors_sync_),
      prev_velocity_(0),
      time_of_update_(0),
      target_velocity_(0),
      prev_index_(0),
      dec_index_(0),
      run_(true),
      nav_calib_(false),
      motors_init_(false),
      motors_ready_(false),
      slip_calculated_(false),
      motors_preoperational_(false),
      motor_failure_(false),
      all_motors_stopped_(false)
{
  state_      = data_.getStateMachineData();
  motor_data_.module_status = data::ModuleStatus::kStart;
  motor_data_.velocity_1 = 0;
  motor_data_.velocity_2 = 0;
  motor_data_.velocity_3 = 0;
  motor_data_.velocity_4 = 0;
  data_.setMotorData(motor_data_);
  motor_velocity_ = {0, 0, 0, 0};
  communicator_ = new Communicator(log);
}

void Main::run()
{
  log_.INFO("MOTOR", "Starting motor controllers");
  System& sys = System::getSystem();
  while (run_ && sys.running_) {
    state_ = data_.getStateMachineData();

    if (state_.current_state == data::State::kIdle) {
      if (!motors_init_) initMotors();
      yield();

    } else if (state_.current_state == data::State::kCalibrating) {
      if (!slip_calculated_) {
        calculateSlip(kAccelerationData);
        calculateSlip(kDecelerationData);
      }
      if (!motors_ready_ && !motor_failure_) prepareMotors();
      yield();

    } else if (state_.current_state == data::State::kReady) {
      // Wait for launch command
      updateMotorData();
      yield();

    } else if (state_.current_state == data::State::kAccelerating) {
      accelerateMotors();

    } else if (state_.current_state == data::State::kDecelerating) {
      decelerateMotors();

    } else if (state_.current_state == data::State::kRunComplete) {
      // Wait for state machine to transition to kExiting
      communicator_->sendTargetVelocity(0);
      updateMotorData();
      yield();

    } else if (state_.current_state == data::State::kExiting) {
      servicePropulsion();

    } else if (state_.current_state == data::State::kEmergencyBraking) {
      stopMotors();

    } else if (state_.current_state == data::State::kFailureStopped) {
      enterPreOperational();
      updateMotorData();
      yield();

    } else {
      run_ = false;
    }
  }
}

void Main::initMotors()
{
  // Register controllers on CAN Bus
  communicator_->registerControllers();

  // Configure controller parameters
  communicator_->configureControllers();

  // If a failure occured during configuration, set motor status to critical failure
  if (communicator_->getFailure()) {
    updateMotorFailure();
  // Otherwise update motor status to initialised
  } else {
    motor_data_.module_status = data::ModuleStatus::kInit;
    data_.setMotorData(motor_data_);
    motors_init_ = true;
    log_.INFO("MOTOR", "Motor State: Idle");
  }
}

void Main::calculateSlip(std::string filepath)
{
  /* Units:
   * Slip - Difference between rotational velocity of wheel and translational velocity of pod.
   *        Positive if rotational velocity > translational velocity,
   *        Negative if rotational velocity < translational velocity,
   *        Zero otherwise.
   *
   * Translational velocity - m/s
   * Angular velocity       - rad/s
   * Radius                 - metres
   */
  double slip, translational_velocity, angular_velocity, rpm;
  std::ifstream data;
  data.open(filepath);

  if (!data.is_open()) {
    log_.ERR("MOTOR", "Could not open file: %s", filepath.c_str());
    updateMotorFailure();
    return;
  } else if (filepath == kAccelerationData) {
    log_.INFO("MOTOR", "Calculating acceleration slip...");
  } else {
    log_.INFO("MOTOR", "Calculating deceleration slip...");
  }

  std::string line;
  while (std::getline(data, line)) {
    std::string split_line;
    std::vector<double> temp_vec;
    std::stringstream ss(line);
    while (std::getline(ss, split_line, '\t')) {
      temp_vec.push_back(std::move(stod(split_line)));
    }

    // Calculate angular velocity from slip, translational velocity and halbach wheel radius
    slip = temp_vec[0];
    translational_velocity = temp_vec[1];
    angular_velocity = (slip + translational_velocity) / kHalbachRadius;

    // Calculate RPM given calculated angular velocity
    rpm = (angular_velocity * 60) / (2*M_PI);

    // Use temporary vector to hold translational velocity and rpm
    temp_vec[0] = translational_velocity;
    temp_vec[1] = rpm;

    // Add data to appropriate container
    if (filepath == kAccelerationData) {
      acceleration_slip_.push_back(temp_vec);
    } else {
      deceleration_slip_.push_back(temp_vec);
    }
  }

  // If both containers have been populated, set bool to true
  if (!acceleration_slip_.empty() && !deceleration_slip_.empty()) {
    acceleration_slip_ = transpose(acceleration_slip_);
    deceleration_slip_ = transpose(deceleration_slip_);
    slip_calculated_ = true;
    log_.INFO("MOTOR", "All slip values calculated");
  }
}

std::vector<std::vector<double>> Main::transpose(std::vector<std::vector<double>> data)
{
  std::vector<std::vector<double>> transpose(2, std::vector<double>(data.size(), 1));
  for (uint16_t i = 0; i < data.size(); ++i) {
    for (uint16_t j = 0; j < 2; ++j) {
      transpose[j][i] = data[i][j];
    }
  }
  return transpose;
}

void Main::prepareMotors()
{
  // Set motors into operational mode
  communicator_->prepareMotors();

  // Check for any errors and warning
  communicator_->healthCheck();

  // If there is an error or warning, set motor status to critical failure
  if (communicator_->getFailure()) {
    updateMotorFailure();
  // Otherwise set motor status to ready
  } else {
    motor_data_.module_status = data::ModuleStatus::kReady;
    data_.setMotorData(motor_data_);
    motors_ready_ = true;
    log_.INFO("MOTOR", "Motor State: Ready");
  }
}

void Main::accelerateMotors()
{
  // Hit the barrier to sync with navigation calibration
  if (!nav_calib_) {
    post_calibration_barrier_.wait();
    nav_calib_ = true;
    log_.INFO("MOTOR", "Motor state: Accelerating");
  }

  while (state_.current_state == data::State::kAccelerating) {
    // Check for motors critical failure flag
    communicator_->healthCheck();

    // If a failure occurs in any motor, set motor status to critical failure
    //  and stop all motors
    if (communicator_->getFailure()) {
      log_.INFO("MOTOR", "Motor failure");
      updateMotorFailure();
      stopMotors();
      break;
    }

    // Otherwise step up motor velocity
    log_.DBG2("MOTOR", "Motor State: Accelerating\n");
    data::Navigation nav_ = data_.getNavigationData();
    target_velocity_      = accelerationVelocity(nav_.velocity);
    communicator_->sendTargetVelocity(target_velocity_);
    updateMotorData();

    // Update state machine data
    state_ = data_.getStateMachineData();
  }
}

void Main::decelerateMotors()
{
  log_.INFO("MOTOR", "Motor State: Deccelerating\n");
  while (state_.current_state == data::State::kDecelerating) {
    // Check for motors critical failure flag
    communicator_->healthCheck();

    // If a failure occurs in any motor, set motor status to critical failure
    //  and stop all motors
    if (communicator_->getFailure()) {
      updateMotorFailure();
      stopMotors();
      break;
    }

    // Otherwise step down motor velocity
    log_.DBG2("MOTOR", "Motor State: Deccelerating\n");
    data::Navigation nav_ = data_.getNavigationData();
    target_velocity_      = decelerationVelocity(nav_.velocity);
    communicator_->sendTargetVelocity(target_velocity_);
    updateMotorData();

    // Update state machine data
    state_ = data_.getStateMachineData();
  }
}

void Main::stopMotors()
{
  // Cut high power to motors in state of emergency
  communicator_->enterPreOperational();

  // Updates the motor data while motors are stopping
  while (!all_motors_stopped_) {
    log_.DBG2("MOTOR", "Motor State: Stopping\n");
    updateMotorData();

    if (motor_velocity_.velocity_1 == 0 && motor_velocity_.velocity_2 == 0 &&
        motor_velocity_.velocity_3 == 0 && motor_velocity_.velocity_4 == 0)
    {
      all_motors_stopped_ = true;
      log_.INFO("MOTOR", "Motor State: Stopped\n");
    }
  }
  updateMotorData();
}

int32_t Main::accelerationVelocity(NavigationType velocity)
{
  // Starting acceleration. TODO(Sean) Check with sims on this value
  if (velocity < 0.5) {
    prev_velocity_ = velocity;
    timer.start();
    time_of_update_ = timer.getTimeMicros();
    return 250;
  }

  // If the current velocity has not exceeded the previous velocity by at least
  // 0.2 m/s in 50 milliseconds, then it is likely that the slip is too low, so
  // we manually increase the RPM.
  if (timer.getTimeMicros() - time_of_update_ > 50000) {
    int32_t rpm = 0;
    if (velocity - prev_velocity_ < 0.2) {
      prev_index_++;
      if (prev_index_ < (int32_t) acceleration_slip_[1].size()) {
        // Increase the velocity to the next RPM
        rpm = (int32_t) acceleration_slip_[1][prev_index_];
      } else {
        // Otherwise we are at max velocity, so return max RPM
        rpm = 6000;
      }
    }
    prev_velocity_ = velocity;
    time_of_update_ = timer.getTimeMicros();
    return rpm;
  }

  // Otherwise, perform upper bound binary search to find the first element in the vector
  // of translational velocities that evaluates to greater than current velocity
  auto upper_bound = std::upper_bound(acceleration_slip_[0].begin(),
                                      acceleration_slip_[0].end(),
                                      velocity);

  // Use index to find corresponding RPM
  if (upper_bound == acceleration_slip_[0].end()) upper_bound--;
  int index       = upper_bound - acceleration_slip_[0].begin();
  prev_index_     = index;
  time_of_update_ = timer.getTimeMicros();
  return (int32_t) acceleration_slip_[1][index];
}

int32_t Main::decelerationVelocity(NavigationType velocity)
{
  // Decrease velocity from max RPM to 0, with updates every 45 milliseconds
  while (timer.getTimeMicros() - time_of_update_ < 45000);
  int32_t rpm;
  time_of_update_ = timer.getTimeMicros();
  if (dec_index_ < (int32_t) deceleration_slip_[1].size()) {
    rpm = (int32_t) deceleration_slip_[1][dec_index_];
    dec_index_++;
  } else {
    // Otherwise return RPM of 0
    return 0;
  }
  // Return calculated RPM if it is greater than 300, otherwise stop motors
  // as we are now slowed down enough for an immediate stop
  return (rpm > 300) ? rpm : 0;
}

void Main::servicePropulsion()
{
  data::Communications comms_ = data_.getCommunicationsData();
  // TODO(Anyone) Check that this is a sufficient velocity
  if (comms_.service_propulsion_go) {
    communicator_->sendTargetVelocity(200);
    updateMotorData();
  } else {
    communicator_->sendTargetVelocity(0);
    updateMotorData();
  }
}

void Main::updateMotorData()
{
  motor_velocity_   = communicator_->requestActualVelocity();
  // Write motor data to data structure
  motor_data_.velocity_1 = motor_velocity_.velocity_1;
  motor_data_.velocity_2 = motor_velocity_.velocity_2;
  motor_data_.velocity_3 = motor_velocity_.velocity_3;
  motor_data_.velocity_4 = motor_velocity_.velocity_4;
  data_.setMotorData(motor_data_);
}

void Main::updateMotorFailure()
{
  log_.ERR("MOTOR", "Motor State: MOTOR FAILURE\n");
  motor_data_ = data_.getMotorData();
  motor_data_.module_status = data::ModuleStatus::kCriticalFailure;
  data_.setMotorData(motor_data_);
  motor_failure_ = true;
}

void Main::enterPreOperational()
{
  if (!motors_preoperational_) {
    communicator_->enterPreOperational();
  }
}

}}  // namespace hyped::motor_control
