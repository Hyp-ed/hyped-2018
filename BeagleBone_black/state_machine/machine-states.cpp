
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

#include "machine-states.hpp"
#include <iostream>

void Idle::react(HypedMachine &machine, Event event) {
  std::cout << "Called Idle React " << std::endl;
  if (event == kOnStart) {
    machine.transition(new Accelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new FailureStopped());
  }
}

void Idle::entry() { std::cout << "Current State: Idle" << std::endl; }

void Accelerating::react(HypedMachine &machine, Event event) {
  std::cout << "Called Accelerating React" << std::endl;
  if (event == kMaxDistanceReached) {
    machine.transition(new Decelerating());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void Accelerating::entry() {
  std::cout << "Current State: Accelerating" << std::endl;
}

void Decelerating::react(HypedMachine &machine, Event event) {
  std::cout << "Called Decelerating React" << std::endl;
  if (event == kEndOfRunReached) {
    machine.transition(new RunComplete());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}

void Decelerating::entry() {
  std::cout << "Current State: Decelerating" << std::endl;
}

void EmergencyBraking::react(HypedMachine &machine, Event event) {
  std::cout << "Called EmergencyBraking React" << std::endl;
  if (event == kVelocityZeroReached) {
    machine.transition(new FailureStopped());
  }
}

void EmergencyBraking::entry() {
  std::cout << "Current State: EmergencyBraking" << std::endl;
}

void FailureStopped::react(HypedMachine &machine, Event event) {
  std::cout << "Called FailureStopped React" << std::endl;
}

void FailureStopped::entry() {
  std::cout << "Current State: FailureStopped" << std::endl;
}

void RunComplete::react(HypedMachine &machine, Event event) {
  std::cout << "Called RunComplete React" << std::endl;
  if (event == kOnExit) {
    machine.transition(new Exiting());
  } else if (event == kCriticalFailure) {
    machine.transition(new FailureStopped());
  }
}

void RunComplete::entry() {
  std::cout << "Current State: RunComplete" << std::endl;
}

void Exiting::entry() { std::cout << "Current State: Exiting" << std::endl; }

void Exiting::react(HypedMachine &machine, Event event) {
  std::cout << "Called Exiting React" << std::endl;
  if (event == kEndOfTubeReached) {
    machine.transition(new Finished());
  } else if (event == kCriticalFailure) {
    machine.transition(new EmergencyBraking());
  }
}
void Finished::entry() { std::cout << "Current State: Finished" << std::endl; }

void Finished::react(HypedMachine &machine, Event event) {
  std::cout << "Called Finished React" << std::endl;
}