/*
 * Author: Sean Mullan
 * Organisation: HYPED
 * Date: 28/03/18
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

#include "motor_control/main.hpp"
#include "utils/concurrent/thread.hpp"
#include "state_machine/hyped-machine.hpp"

#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;
using hyped::utils::System;
using hyped::state_machine::HypedMachine;

using namespace hyped;

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System& sys = System::getSystem();
  Logger log_motor(sys.verbose_motor, sys.debug_motor);
  Logger log_state(sys.verbose_state, sys.debug_state);

  Logger log(true, 1);
  HypedMachine* hypedMachine = new HypedMachine(log);
  Thread* motor  = new hyped::motor_control::Main(1, log);
  motor->start();

  Thread::sleep(5000);
  hypedMachine->handleEvent(state_machine::Event::kInitialised);
  Thread::sleep(5000);
  hypedMachine->handleEvent(state_machine::Event::kSystemsChecked);
  hypedMachine->handleEvent(state_machine::Event::kOnStart);
  Thread::sleep(10000);
  hypedMachine->handleEvent(state_machine::Event::kMaxDistanceReached);
  Thread::sleep(5000);
  hypedMachine->handleEvent(state_machine::Event::kVelocityZeroReached);
  hypedMachine->handleEvent(state_machine::Event::kOnExit);

  motor->join();

  delete motor;

  return 0;
}
