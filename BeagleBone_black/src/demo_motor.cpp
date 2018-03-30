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

#include <iostream>
#include "motor_control/main.hpp"
#include "motor_control/test-states.hpp"
#include "utils/concurrent/thread.hpp"

using hyped::utils::concurrent::Thread;

int main()
{
  Thread* states = new hyped::motor_control::TestStates(0);
  Thread* motor  = new hyped::motor_control::Main(1);

  states->start();
  motor->start();

  states->join();
  motor->join();

  delete states;
  delete motor;

  return 0;
}
