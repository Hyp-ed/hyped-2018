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
    data_(data::Data::getInstance()),
    comms_data(data_.getCommunicationsData()),
    nav_data(data_.getNavigationData()),
    sm_data(data_.getStateMachineData())
{ /* EMPTY */ }

/**
  *  @brief  Runs state machine thread.
  */

void Main::run()
{
  while (1) {
    checkCommunications();
    checkNavigation();
    checkReady();
  }
}

void Main::checkNavigation()
{
/**
  *  @TODO Check if margin (20m) is appropriate
  */

if((nav_data.distance + nav_data.emergency_braking_distance) + 20 >= comms_data.run_length)
{
hypedMachine.handleEvent(kCriticalFailure);
}

if(nav_data.state == data::NavigationState::kCriticalFailure)
{
  hypedMachine.handleEvent(kCriticalFailure);
}
}

void Main::checkCommunications()
{
  if (comms_data.stopCommand) {
    hypedMachine.handleEvent(kCriticalFailure);
  }

  if (comms_data.launchCommand) {
    hypedMachine.handleEvent(kOnStart);
  }

  if (comms_data.resetCommand) {
    hypedMachine.reset();
  }
}
//  @TODO add checks for other modules' states
void Main::checkReady()
{
  if (nav_data.state == data::NavigationState::kReady) {
    hypedMachine.handleEvent(kSystemsChecked);
  }
}

}}  // namespace hyped::state_machine
