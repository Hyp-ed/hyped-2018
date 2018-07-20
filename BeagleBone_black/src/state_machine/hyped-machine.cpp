
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

namespace hyped {
namespace state_machine {

GPIO* HypedMachine::pin_embrake_ = nullptr;
GPIO* HypedMachine::pin_water_   = nullptr;

HypedMachine::HypedMachine(utils::Logger& log)
    : current_state_(State::alloc_)
    , log_(log)
{
  log_.INFO("STATE", "State Machine initialised");
  transition(new(current_state_) Idle());
}

void HypedMachine::handleEvent(Event event)
{
  log_.DBG1("STATE", "Raised event %d", event);
  current_state_->react(*this, event);
}

void HypedMachine::transition(State *state)
{
  // NOTE, no use of argument state, as all react() functions allocate all new
  // states directly to current_state_ variable through common State::alloc_ pointer
  current_state_->entry();
  log_.INFO("STATE", "Transitioned to %s"
    , data::states[current_state_->state_]);
  state_machine_.current_state = current_state_->state_;

  // update shared data structure
  static data::Data& d = data::Data::getInstance();
  d.setStateMachineData(state_machine_);
}

void HypedMachine::reset()
{
  log_.INFO("STATE", "State Machine resetted");
  transition(new(current_state_) Idle());
}

void HypedMachine::setupEmbrakes()
{
  pin_embrake_ = new GPIO(46, utils::io::gpio::Direction::kOut);
  pin_water_   = new GPIO(47, utils::io::gpio::Direction::kOut);
  pin_embrake_->set();
  pin_water_->set();
}

void HypedMachine::engageEmbrakes()
{
  utils::Logger log(true, 0);
  if (pin_embrake_) {
    pin_embrake_->clear();
    log.INFO("STATE", "Emergency brakes engaged");
  } else {
    log.INFO("STATE", "Emergency brakes not initialised, we are going to die");
  }
}

}}   // namespace hyped::state_machine
