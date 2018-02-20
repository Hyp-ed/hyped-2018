
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

#include <algorithm>
#include <iostream>
#include <list>
#include <mutex>
#include <thread>
//#include "motor_control/main_motor.cpp"
#include "state_machine/main_machine.cpp"

// global variable for shared data
std::list<int> hypedList;

// a global instance of std::mutex to protect global variable
std::mutex hypedMutex;

/*
Before using the shared data use the following instead of lock() and unlock():
              std::lock_guard<std::mutex> guard(myMutex);
*/

int main() {
  std::cout << "Starting BeagleBone Black..." << std::endl;
  std::thread t1(hyped::state_machine::main);
  t1.join();
  std::cout << "Started State Machine" << std::endl;

  return 0;
}
