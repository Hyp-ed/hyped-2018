/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 16/05/18
 * Description: Demo for VL6180 sensor
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


#include "sensors/vl6180.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::sensors::VL6180;
using hyped::utils::Logger;

Logger log(true, 1);

// Not ready yet!
int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  VL6180 vl6180 = VL6180(0x29, 1, log);

  log.INFO("TEST-vl6180", "VL6180 instance successfully created");

  Thread::sleep(5000);

  double distance = vl6180.getDistance();

  log.INFO("TEST-vl6180", "Distance: %f", distance);

 	return 0;
}
