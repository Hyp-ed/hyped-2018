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
#include "utils/timer.hpp"
#include "utils/system.hpp"

namespace hyped {
namespace state_machine {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      hypedMachine(log),
      timeout_(60000000),     // 60 seconds
      data_(data::Data::getInstance())
{ /* EMPTY */ }

/**
  *  @brief  Runs state machine thread.
  */

void Main::run()
{
  utils::System& sys = utils::System::getSystem();
  while (sys.running_) {
    comms_data_     = data_.getCommunicationsData();
    nav_data_       = data_.getNavigationData();
    sm_data_        = data_.getStateMachineData();
    motor_data_     = data_.getMotorData();
    batteries_data_ = data_.getBatteriesData();
    // sensors_data_   = data_.getSensorsData();

    switch (sm_data_.current_state) {
      case data::State::kIdle:
        if (checkCommsCriticalFailure()) break;   // TODO(anyone): discuss this transition again
        if (checkInitialised())          break;
        break;
      case data::State::kCalibrating:
        if (checkCriticalFailure())      break;
        if (checkSystemsChecked())       break;
        break;
      case data::State::kReady:
        if (checkCriticalFailure())      break;
        if (checkOnStart())              break;
        break;
      case data::State::kAccelerating:
        if (checkCriticalFailure())      break;
        if (checkTimer())                break;
        if (checkMaxDistanceReached())   break;
        break;
      case data::State::kDecelerating:
        if (checkCriticalFailure())      break;
        if (checkTimer())                break;
        if (checkVelocityZeroReached())  break;
        break;
      case data::State::kRunComplete:
        if (checkCriticalFailure())      break;
        if (checkOnExit())               break;
        break;
      case data::State::kExiting:
        if (checkCriticalFailure())      break;
        if (checkFinish())               break;
        break;
      case data::State::kEmergencyBraking:
        if (checkVelocityZeroReached())  break;
        break;
      // we cannot recover from these states
      case data::State::kInvalid:
        log_.ERR("STATE", "we are in Invalid state");
      case data::State::kFinished:
      case data::State::kFailureStopped:
      default:
        break;
    }

    yield();
  }
}

bool Main::checkInitialised()
{
  // all modules must be initialised
  if (comms_data_.module_status     == data::ModuleStatus::kInit &&
      nav_data_.module_status       == data::ModuleStatus::kInit &&
      motor_data_.module_status     == data::ModuleStatus::kInit &&
      // sensors_data_.module_status   == data::ModuleStatus::kInit &&
      batteries_data_.module_status == data::ModuleStatus::kInit) {
    log_.INFO("STATE", "all modules are initialised");
    hypedMachine.handleEvent(kInitialised);
    return true;
  }
  return false;
}

bool Main::checkSystemsChecked()
{
  // nav and motors must be ready
  if (nav_data_.module_status   == data::ModuleStatus::kReady &&
      motor_data_.module_status == data::ModuleStatus::kReady) {
    log_.INFO("STATE", "systems ready");
    hypedMachine.handleEvent(kSystemsChecked);
    return true;
  }
  return false;
}

bool Main::checkOnStart()
{
  if (comms_data_.launch_command) {
    log_.INFO("STATE", "launch command received");
    hypedMachine.handleEvent(kOnStart);

    // also setup timer for going to emergency braking state
    time_start_ = utils::Timer::getTimeMicros();
    return true;
  }
  return false;
}

bool Main::checkCommsCriticalFailure()
{
  if (comms_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.ERR("STATE", "Critical failure caused by communications ");
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }
  return false;
}

bool Main::checkCriticalFailure()
{
  // check if any of the module has failed (except sensors)
  if (comms_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.ERR("STATE", "Critical failure caused by communications ");
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }
  if (nav_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.ERR("STATE", "Critical failure caused by navigation ");
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }
  if (motor_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.ERR("STATE", "Critical failure caused by motors ");
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }
  if (batteries_data_.module_status == data::ModuleStatus::kCriticalFailure) {
    log_.ERR("STATE", "Critical failure caused by batteries ");
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }

  // also check if emergency braking distance has been reached
  if (nav_data_.distance +
      nav_data_.emergency_braking_distance +
      20 >= comms_data_.run_length) {
    log_.ERR("STATE", "Critical failure, emergency braking distance reached");
    log_.ERR("STATE", "current distance, emergency distance: %f %f"
      , nav_data_.distance
      , comms_data_.run_length - nav_data_.emergency_braking_distance);
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }
  return false;
}

bool Main::checkMaxDistanceReached()
{
  if (nav_data_.distance +
      nav_data_.braking_distance +
      20 >= comms_data_.run_length) {
    log_.INFO("STATE", "Max distance reached");
    log_.INFO("STATE", "current distance, braking distance: %f %f"
      , nav_data_.distance
      , comms_data_.run_length - nav_data_.braking_distance);
    hypedMachine.handleEvent(kMaxDistanceReached);
    return true;
  }
  return false;
}

bool Main::checkOnExit()
{
  if (comms_data_.service_propulsion_go) {
    log_.INFO("STATE", "initialising service propulsion");
    hypedMachine.handleEvent(kOnExit);
    return true;
  }
  return false;
}

bool Main::checkFinish()
{
  // no implementation yet
  return false;
}

bool Main::checkVelocityZeroReached()
{
  if (motor_data_.velocity_1 == 0 && motor_data_.velocity_2 == 0
      && motor_data_.velocity_3 == 0 && motor_data_.velocity_4 == 0) {
    log_.INFO("STATE", "RPM reached zero.");
    hypedMachine.handleEvent(kVelocityZeroReached);
    return true;
  }
  return false;
}

bool Main::checkTimer()
{
  if (utils::Timer::getTimeMicros() > time_start_ + timeout_) {
    log_.ERR("STATE", "Timer expired");
    hypedMachine.handleEvent(kCriticalFailure);
    return true;
  }
  return false;
}

}}  // namespace hyped::state_machine
