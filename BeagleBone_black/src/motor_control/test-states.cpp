/*
 * Author: Sean Mullan
 * Organisation: HYPED
 * Date: 28. March 2018
 * Description: Dummy state machine thread to test if motor control module reacts
 *              appropriately to state changes
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
#include "motor_control/test-states.hpp"

namespace hyped {
namespace motor_control {

TestStates::TestStates(uint8_t id, Logger& log)
    : Thread(id, log)
{
  hypedMachine = new HypedMachine(log);
}

/**
  *  @brief  Runs test state machine thread
  */
void TestStates::run()
{
  std::cout << "State machine thread successfully started" << std::endl;
  hypedMachine->handleEvent(state_machine::Event::kOnStart);
  delay(5000);
  hypedMachine->handleEvent(state_machine::Event::kMaxDistanceReached);
  delay(2500);
  hypedMachine->handleEvent(state_machine::Event::kCriticalFailure);
  delay(500);
  hypedMachine->handleEvent(state_machine::Event::kVelocityZeroReached);
  delay(1000);
  // hypedMachine->handleEvent(kEndOfRunReached);
  // delay(10000);
  // hypedMachine->handleEvent(kOnExit);
  // hypedMachine->handleEvent(kEndOfTubeReached);
}

/**
  *  @brief  Delays thread
  */
void TestStates::delay(int i)
{
  while (i--);
}

}}
