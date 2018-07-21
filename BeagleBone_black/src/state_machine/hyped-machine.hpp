
/*
 * Authors: Yash Mittal and Ragnor Comerford
 * Organisation: HYPED
 * Date: 11. February 2018
 * Description:
 * HypedMachine wraps around State objects, i.e. functions as a State manager.
 * HypedMachine reacts to events through handleEvent() function. Fucntion transition()
 * finalises changes of current state and facilitate updates to the shared data structure.
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
#ifndef BEAGLEBONE_BLACK_STATE_MACHINE_HYPED_MACHINE_HPP_
#define BEAGLEBONE_BLACK_STATE_MACHINE_HYPED_MACHINE_HPP_

#include "data/data.hpp"

#include "state_machine/event.hpp"
#include "state_machine/states.hpp"

#include "utils/logger.hpp"
#include "utils/io/gpio.hpp"
namespace hyped {
using utils::io::GPIO;
namespace state_machine {

class State;
class HypedMachine {
 public:
  explicit HypedMachine(utils::Logger& log);
  void handleEvent(Event event);
  void transition(State *state);
  void reset();

  static void setupEmbrakes();
  static void engageEmbrakes();

 private:
  State*             current_state_;
  utils::Logger&     log_;
  data::StateMachine state_machine_;

  static GPIO* pin_embrake_;
  static GPIO* pin_water_;
};

}}   // namespace hyped::state_machine

#endif  // BEAGLEBONE_BLACK_STATE_MACHINE_HYPED_MACHINE_HPP_
