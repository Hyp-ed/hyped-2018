
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
#include "state_machine/states.hpp"
#include <stdlib.h>

namespace hyped {

using state = data::State;
using utils::io::GPIO;
using utils::System;

namespace state_machine {

// statically allocate memory for current_state
State* State::alloc_ = static_cast<State*>(malloc(sizeof(State)));

void Idle::entry()
{
  state_ = state::kIdle;
}

void Idle::react(HypedMachine &machine, Event event)
{
  if (event == kInitialised) {
    machine.transition(new(alloc_) Calibrating());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) FailureStopped());
  }
}

void Calibrating::entry()
{
  state_ = state::kCalibrating;
}

void Calibrating::react(HypedMachine &machine, Event event)
{
  if (event == kSystemsChecked) {
    machine.transition(new(alloc_) Ready());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) FailureStopped());
  }
}

void Ready::entry()
{
  state_ = state::kReady;
}

void Ready::react(HypedMachine &machine, Event event)
{
  if (event == kOnStart) {
     machine.transition(new(alloc_) Accelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) FailureStopped());
  }
}

void Accelerating::entry()
{
  state_ = state::kAccelerating;
}

void Accelerating::react(HypedMachine &machine, Event event)
{
  if (event == kMaxDistanceReached) {
    machine.transition(new(alloc_) Decelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) EmergencyBraking());
  }
}

void Decelerating::entry()
{
  state_ = state::kDecelerating;
}

void Decelerating::react(HypedMachine &machine, Event event)
{
  if (event == kVelocityZeroReached) {
    machine.transition(new(alloc_) RunComplete());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) EmergencyBraking());
  }
}

void EmergencyBraking::entry()
{
  HypedMachine::engageEmbrakes();
  state_ = state::kEmergencyBraking;
}

void EmergencyBraking::react(HypedMachine &machine, Event event)
{
  if (event == kVelocityZeroReached) {
    machine.transition(new(alloc_) FailureStopped());
  }
}

void FailureStopped::entry()
{
  HypedMachine::engageEmbrakes();
  state_ = state::kFailureStopped;
}

void FailureStopped::react(HypedMachine &machine, Event event)
{
}

void RunComplete::entry()
{
  state_ = state::kRunComplete;
}

void RunComplete::react(HypedMachine &machine, Event event)
{
  if (event == kOnExit) {
    machine.transition(new(alloc_) Exiting());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) FailureStopped());
  }
}

void Exiting::entry()
{
  state_ = state::kExiting;
}

void Exiting::react(HypedMachine &machine, Event event)
{
  if (event == kFinish) {
    machine.transition(new(alloc_) Finished());
  } else if (event == kCriticalFailure) {
    machine.transition(new(alloc_) EmergencyBraking());
  }
}

void Finished::entry()
{
  state_ = state::kFinished;
}

void Finished::react(HypedMachine &machine, Event event)
{
}


}}   // namespace hyped::state_machine
