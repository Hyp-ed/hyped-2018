
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


#include "state_machine/main.hpp"
#include "motor_control/main.hpp"
#include "navigation/main.hpp"
#include "sensors/main.hpp"
#include "utils/concurrent/thread.hpp"

#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;
using hyped::utils::System;

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System sys = System::getSystem();
  Logger log(sys.verbose, sys.debug);
  // std::cout << "Starting BeagleBone Black and initialising threads..." << std::endl;
  log.INFO("[MAIN]: Starting BBB with %d modules\n", 4);
  log.DBG("[MAIN]: DBG\n");
  log.DBG0("[MAIN]: DBG0\n");
  log.DBG1("[MAIN]: DBG1\n");
  log.DBG2("[MAIN]: DBG2\n");
  log.DBG3("[MAIN]: DBG3\n");

  log.INFO("argc %d argv[0] %s\n", argc, argv[0]);
  Thread* state_machine   = new hyped::state_machine::Main(0, log);
  Thread* motor     = new hyped::motor_control::Main(1, log);
  Thread* sensors   = new hyped::sensors::Main(2, log);
  Thread* navigation = new hyped::navigation::Main(3, log);

  state_machine->start();
  motor->start();
  sensors->start();
  navigation->start();

  state_machine->join();
  motor->join();
  sensors->join();
  navigation->join();

  delete state_machine;
  delete sensors;
  delete motor;
  delete navigation;

  return 0;
}
