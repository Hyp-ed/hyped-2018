
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

#include "state_machine/event.hpp"
#include "state_machine/hyped-machine.hpp"
#include "state_machine/states.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"

using namespace hyped::state_machine;
using hyped::utils::System;

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  hyped::utils::Logger log(true, 1);
  HypedMachine hypedMachine(log);
  hypedMachine.handleEvent(kInitialised);
  hypedMachine.handleEvent(kSystemsChecked);
  hypedMachine.handleEvent(kOnStart);
  hypedMachine.handleEvent(kMaxDistanceReached);
  hypedMachine.handleEvent(kVelocityZeroReached);
  hypedMachine.handleEvent(kOnExit);
  return 0;
}
