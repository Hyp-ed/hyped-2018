
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

#include <iostream>

#include "motor_control/main.hpp"
#include "utils/concurrent/thread.hpp"


using hyped::utils::concurrent::Thread;

int main()
{
  std::cout << "Starting BeagleBone Black..." << std::endl;

  Thread* motor = new hyped::motor_control::Main(0);

  motor->start();

  motor->join();
  delete motor;
  return 0;
}
