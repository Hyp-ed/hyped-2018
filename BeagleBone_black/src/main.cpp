
/*
 * Authors : HYPED
 * Organisation: HYPED
 * Date: 3. February 2018
 * Description:
 * This is the main executable for BeagleBone pod node
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

#include "state_machine/main.hpp"
#include "motor_control/main.hpp"
#include "navigation/main.hpp"
#include "sensors/main.hpp"
#include "utils/concurrent/thread.hpp"

#include "utils/logger.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;

int main()
{
  Logger log(true, 2);
  // std::cout << "Starting BeagleBone Black and initialising threads..." << std::endl;
  log.INFO("[MAIN]: Starting BBB with %d modules\n", 4);
  log.DBG("[MAIN]: DBG\n");
  log.DBG0("[MAIN]: DBG0\n");
  log.DBG1("[MAIN]: DBG1\n");
  log.DBG2("[MAIN]: DBG2\n");
  log.DBG3("[MAIN]: DBG3\n");
  Thread* state_machine   = new hyped::state_machine::Main(0);
  Thread* motor     = new hyped::motor_control::Main(1);
  Thread* sensors   = new hyped::sensors::Main(2);
  Thread* navigation = new hyped::navigation::Main(3);

  // state_machine->start();
  // motor->start();
  // sensors->start();
  // navigation->start();

  // state_machine->join();
  // motor->join();
  // sensors->join();
  // navigation->join();

  delete state_machine;
  delete sensors;
  delete motor;
  delete navigation;

  return 0;
}
