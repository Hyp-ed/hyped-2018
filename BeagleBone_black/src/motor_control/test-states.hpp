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

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_TEST_STATES_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_TEST_STATES_HPP_

#include <cstdint>
#include "utils/concurrent/thread.hpp"
#include "state_machine/hyped-machine.hpp"

namespace hyped {

using utils::concurrent::Thread;
using hyped::state_machine::HypedMachine;

namespace motor_control {

class TestStates: public Thread {
public:
 explicit TestStates(uint8_t id);
 void run() override;
 void delay(int i);

private:
 HypedMachine* hypedMachine;
};

}}  // namespace hyped::motor_control

#endif
