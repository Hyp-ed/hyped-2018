/*
 * Author: Sean Mullan and Jack Horsburgh
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

#include "motor_control/controller.hpp"

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include <iostream>
#include <fstream>
using namespace std;

using hyped::utils::Logger;
using hyped::utils::System;
using hyped::utils::concurrent::Thread;

using namespace hyped;

using hyped::motor_control::Controller;

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System& sys = System::getSystem();
  Logger log_motor(sys.verbose_motor, sys.debug_motor);

  Logger log(true, 1);
  Controller* controller = new Controller(log,1);

  ofstream myfile;
  myfile.open ("RPMvTime.txt");
  myfile << "RPM\tTime\n";

  controller->registerController();
  controller->configure();
  controller->enterOperational();
  int32_t target_v = 0;
  int32_t actual_v = 0;
  for (int i = 0; i < 250; i++) {
    if (i < 170) {
      controller->sendTargetVelocity(target_v);
      target_v += 41;
    } else {
      controller->quickStop();
    }
    controller->updateActualVelocity();
    actual_v = controller->getVelocity();
    myfile << actual_v<<"\t"<<i<<"\n";
    Thread::sleep(100);
  }
  myfile.close();
}
