
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
  Logger log_system(sys.verbose, sys.debug);
  Logger log_motor(sys.verbose_motor, sys.debug_motor);
  Logger log_nav(sys.verbose_nav, sys.debug_nav);
  Logger log_sensor(sys.verbose_sensor, sys.debug_sensor);
  Logger log_state(sys.verbose_state, sys.debug_state);

  log_system.INFO("[MAIN]: Starting BBB with %d modules\n", 4);
  log_system.DBG("[MAIN]: DBG\n");
  log_system.DBG0("[MAIN]: DBG0\n");
  log_system.DBG1("[MAIN]: DBG1\n");
  log_system.DBG2("[MAIN]: DBG2\n");
  log_system.DBG3("[MAIN]: DBG3\n");

  Thread* state_machine   = new hyped::state_machine::Main(0, log_state);
  Thread* motor     = new hyped::motor_control::Main(1, log_motor);
  Thread* sensors   = new hyped::sensors::Main(2, log_sensor);
  Thread* navigation = new hyped::navigation::Main(3, log_nav);

  state_machine->start();
  motor->start();
  sensors->start();
  navigation->start();
  log_system.INFO("[MAIN]: all module threads started\n");
  // Thread::sleep(1000);
  log_system.INFO("[MAIN]: After 1 sec sleep\n");

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
