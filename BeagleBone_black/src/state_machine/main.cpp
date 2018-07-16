/*
 * Author: Ragnor Comerford
 * Organisation: HYPED
 * Date: 11. March 2018
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



#include <cstdint>

#include "state_machine/hyped-machine.hpp"
#include "state_machine/main.hpp"

#include "data/data.hpp"

namespace hyped {
namespace state_machine {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      hypedMachine(log),
      data_(data::Data::getInstance())
{ /* EMPTY */ }

/**
  *  @brief  Runs state machine thread.
  */

void Main::run()
{
  while (1) {
    comms_data_     = data_.getCommunicationsData();
    nav_data_       = data_.getNavigationData();
    sm_data_        = data_.getStateMachineData();
    motor_data_     = data_.getMotorData();
    batteries_data_ = data_.getBatteriesData();
    sensors_data_   = data_.getSensorsData();

    if (sm_data_.current_state == data::kIdle) {
      checkInit();
    }
    if (sm_data_.current_state != data::kEmergencyBraking
        && sm_data_.current_state != data::kFailureStopped) {
      checkFailure();
    }
    checkReady();
    if (sm_data_.current_state != data::kIdle) {
      checkNavigation();
      checkCommunications();
    }
    yield();
  }
}

void Main::checkNavigation()
{
/**
  *  @TODO Check if margin (20m) is appropriate
  */

  if ((sm_data_.current_state == data::kDecelerating
      || sm_data_.current_state == data::kAccelerating)
      && ((nav_data_.distance + nav_data_.emergency_braking_distance)
      + 20 >= comms_data_.run_length)) {
    log_.INFO("STATE", "Critical failure caused by exceeding emergency braking distance.");
    hypedMachine.handleEvent(kCriticalFailure);
  }

  if ((sm_data_.current_state == data::kDecelerating
      || sm_data_.current_state == data::kAccelerating)
      && ((nav_data_.distance + nav_data_.braking_distance)
      + 20 >= comms_data_.run_length)) {
    log_.INFO("STATE", "Critical failure caused by exceeding braking distance.");
    hypedMachine.handleEvent(kMaxDistanceReached);
  }

  if ((sm_data_.current_state == data::kDecelerating
       || sm_data_.current_state == data::kEmergencyBraking) && nav_data_.velocity <= 0.01) {
    log_.INFO("STATE", "Velocity reached zero.");
    hypedMachine.handleEvent(kVelocityZeroReached);
  }
}

void Main::checkCommunications()
{
  if (sm_data_.current_state == data::kReady && comms_data_.launch_command) {
    hypedMachine.handleEvent(kOnStart);
    log_.INFO("STATE", "State machine received launch command");
  }

  if (comms_data_.reset_command) {
    hypedMachine.reset();
    log_.INFO("STATE", "State machine received reset command");
    comms_data_.reset_command = false;
    data_.setCommunicationsData(comms_data_);
  }
}

void Main::checkFailure()
{
  if (comms_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.INFO("STATE", "Critical failure caused by communications ");
    hypedMachine.handleEvent(kCriticalFailure);
  }
  if (nav_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.INFO("STATE", "Critical failure caused by navigation ");
    hypedMachine.handleEvent(kCriticalFailure);
  }
  if (motor_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.INFO("STATE", "Critical failure caused by motors ");
    hypedMachine.handleEvent(kCriticalFailure);
  }
  if (sensors_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.INFO("STATE", "Critical failure caused by sensors ");
    hypedMachine.handleEvent(kCriticalFailure);
  }
  if (batteries_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.INFO("STATE", "Critical failure caused by batteries ");
    hypedMachine.handleEvent(kCriticalFailure);
  }
}

//  @TODO add checks for other modules' states
void Main::checkReady()
{
  if (nav_data_.module_status == data::ModuleStatus::kReady
      &&  motor_data_.module_status == data::ModuleStatus::kReady) {
    hypedMachine.handleEvent(kSystemsChecked);  // Transitions to Ready
  }
}

void Main::checkInit()
{
  if (comms_data_.module_status == data::ModuleStatus::kInit
      && nav_data_.module_status == data::ModuleStatus::kInit
      && motor_data_.module_status == data::ModuleStatus::kInit
      && sensors_data_.module_status == data::ModuleStatus::kInit
      && batteries_data_.module_status == data::ModuleStatus::kInit) {
    hypedMachine.handleEvent(kInitialised);  // Transitions to Calibrating
  }
}



}}  // namespace hyped::state_machine
