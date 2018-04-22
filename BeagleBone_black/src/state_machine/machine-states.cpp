
/*
 * Authors: Yash Mittal and Ragnor Comerford
 * Organisation: HYPED
 * Date: 11. February 2018
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
#include "state_machine/machine-states.hpp"
#include "data/data.hpp"


namespace hyped {
namespace state_machine {

void Idle::entry()
{
  updateData(data::kIdle);
  log_.DBG1("STATE", "Entered Idle");

  // std::cout << "Entered state 'Idle'" << std::endl;
}

void Idle::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'Idle' is now reacting to an event" << std::endl;
  if (event == kOnStart) {
    machine.transition(new Accelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new FailureStopped());
  }
}

void Accelerating::entry()
{
  updateData(data::kAccelerating);
  log_.DBG1("STATE", "Entered Accelerating");
}

void Accelerating::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'Accelerating' is now reacting to an event" << std::endl;
  if (event == kMaxDistanceReached) {
    machine.transition(new Decelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void Decelerating::entry()
{
  updateData(data::kDecelerating);
  log_.DBG1("STATE", "Entered Decelerating");
}

void Decelerating::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'Decelerating' is now reacting to an event" << std::endl;
  if (event == kEndOfRunReached) {
    machine.transition(new RunComplete());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void EmergencyBraking::entry()
{
  updateData(data::kEmergencyBraking);
  log_.DBG1("STATE", "Entered EmergencyBraking");
}

void EmergencyBraking::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'EmergencyBraking' is now reacting to an event" << std::endl;
  if (event == kVelocityZeroReached) {
    machine.transition(new FailureStopped());
  }
}

void FailureStopped::entry()
{
  updateData(data::kFailureStopped);
  log_.DBG1("STATE", "Entered FailureStopped");
}

void FailureStopped::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'FailureStopped' is now reacting to an event" << std::endl;
}

void RunComplete::entry()
{
  updateData(data::kRunComplete);
  log_.DBG1("STATE", "Entered RunComplete");
}

void RunComplete::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'RunComplete' is now reacting to an event" << std::endl;
  if (event == kOnExit) {
    machine.transition(new Exiting());
  } else if (event == kCriticalFailure) {
    machine.transition(new FailureStopped());
  }
}

void Exiting::entry()
{
  updateData(data::kExiting);
  log_.DBG1("STATE", "Entered Exiting");
}

void Exiting::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'Exiting' is now reacting to an event" << std::endl;
  if (event == kEndOfTubeReached) {
    machine.transition(new Finished());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void Finished::entry()
{
  updateData(data::kFinished);
  log_.DBG1("STATE", "Entered Finished");
}

void Finished::react(HypedMachine &machine, Event event)
{
  // std::cout << "State 'Finished' is now reacting to an event" << std::endl;
}


void State::updateData(data::State s)
{
  static data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = s;
  data.setStateMachineData(stm_data);
}
}}   // namespace hyped::state_machine
