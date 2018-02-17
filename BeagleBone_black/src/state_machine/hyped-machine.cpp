
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

#include "state_machine/hyped-machine.hpp"
#include <iostream>

HypedMachine::HypedMachine() : current_state(new Idle()) {
  current_state->entry();
}

void HypedMachine::handleEvent(Event event) {
  std::cout << "Raised event " << event << std::endl;
  current_state->react(*this, event);
}

void HypedMachine::transition(State *state) {
  // State *prevState = currState;
  current_state = state;
  std::cout << "Transitioning..." << std::endl;
  current_state->entry();
  // delete prevState;
}