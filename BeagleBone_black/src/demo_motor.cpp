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

// #include <iostream>
#include "motor_control/main.hpp"
#include "utils/concurrent/thread.hpp"
#include "state_machine/hyped-machine.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;
using hyped::state_machine::HypedMachine;
using namespace hyped;

int main()
{
  Logger log(true, 1);
  HypedMachine* hypedMachine = new HypedMachine(log);
  Thread* motor  = new hyped::motor_control::Main(1, log);
  motor->start();

  log.INFO("TEST", "State machine thread successfully started");
  hypedMachine->handleEvent(state_machine::Event::kOnStart);
  Thread::sleep(500);
  hypedMachine->handleEvent(state_machine::Event::kMaxDistanceReached);
  Thread::sleep(250);
  hypedMachine->handleEvent(state_machine::Event::kCriticalFailure);
  Thread::sleep(50);
  hypedMachine->handleEvent(state_machine::Event::kVelocityZeroReached);
  Thread::sleep(100);

  motor->join();

  delete motor;

  return 0;
}
