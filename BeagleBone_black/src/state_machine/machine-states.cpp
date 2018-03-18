
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
#include <iostream>

namespace hyped {
namespace state_machine {

void Idle::entry() 
{ 
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kIdle;
  data.setStateMachineData(stm_data);
  
  std::cout << "Entered state 'Idle'" << std::endl; 
}

void Idle::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'Idle' is now reacting to an event" << std::endl;
  if (event == kOnStart) {
    machine.transition(new Accelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new FailureStopped());
  }
}

void Accelerating::entry()
{
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kAccelerating;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'Accelerating'" << std::endl;
}

void Accelerating::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'Accelerating' is now reacting to an event" << std::endl;
  if (event == kMaxDistanceReached) {
    machine.transition(new Decelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void Decelerating::entry()
{
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kDecelerating;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'Decelerating'" << std::endl;
}

void Decelerating::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'Decelerating' is now reacting to an event" << std::endl;
  if (event == kEndOfRunReached) {
    machine.transition(new RunComplete());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void EmergencyBraking::entry()
{
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kEmergencyBraking;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'EmergencyBraking'" << std::endl;
}

void EmergencyBraking::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'EmergencyBraking' is now reacting to an event" << std::endl;
  if (event == kVelocityZeroReached) {
    machine.transition(new FailureStopped());
  }
}

void FailureStopped::entry()
{
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kFailureStopped;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'FailureStopped'" << std::endl;
}

void FailureStopped::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'FailureStopped' is now reacting to an event" << std::endl;
}

void RunComplete::entry()
{
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kRunComplete;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'RunComplete'" << std::endl;
}

void RunComplete::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'RunComplete' is now reacting to an event" << std::endl;
  if (event == kOnExit) {
    machine.transition(new Exiting());
  } else if (event == kCriticalFailure) {
    machine.transition(new FailureStopped());
  }
}

void Exiting::entry()
{ 
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kExiting;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'Exiting'" << std::endl;
}

void Exiting::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'Exiting' is now reacting to an event" << std::endl;
  if (event == kEndOfTubeReached) {
    machine.transition(new Finished());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void Finished::entry() 
{ 
  data::Data& data = data::Data::getInstance();
  data::StateMachine stm_data = data.getStateMachineData();
  stm_data.current_state = data::kFinished;
  data.setStateMachineData(stm_data);
  std::cout << "Entered state 'Finished'" << std::endl; 
}

void Finished::react(HypedMachine &machine, Event event)
{
  std::cout << "State 'Finished' is now reacting to an event" << std::endl;
}

}}   // namespace hyped::state_machine
