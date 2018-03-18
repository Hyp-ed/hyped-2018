/*
 * Author: Ragnor Comerford
 * Organisation: HYPED
 * Date: 11. March 2018
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



#include <cstdint>
#include <iostream>

#include "state_machine/hyped-machine.hpp"
#include "state_machine/main.hpp"

#include "data/data.hpp"

namespace hyped {
namespace state_machine {

Main::Main(uint8_t id)
    : Thread(id)
{
  hypedMachine = new HypedMachine;
}

/**
  *  @brief  Runs state machine thread. 
  */

void Main::run()
{
  std::cout << "State machine thread successfully started" << std::endl;

  while (1) {
   data::Navigation nav_data = data.getNavigationData();

   if (hasCriticalFailure()) {
         hypedMachine->handleEvent(kCriticalFailure);
   }
   if (hasReachedMaxDistance()) {
       hypedMachine->handleEvent(kMaxDistanceReached);
   }
  
  }
}

 /**
   * @brief      Checks all the system for critical failures. Returns false if all checked have passed.
   */

bool Main::hasCriticalFailure()
{
return false;
}

 /**
   * @brief      Computes the maxDistance and returns true if it has been reached.
   */

bool Main::hasReachedMaxDistance()
{
  return false;
}

}}  // namespace hyped::state_machine
