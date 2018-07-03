/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 30/05/18
 * Description: Demo for MPU9250 sensor
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

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "sensors/bms.hpp"
#include "data/data.hpp"

using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;

using hyped::data::Battery;
using hyped::sensors::BMSHP;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();
  BMSHP bms(1, log);

  Battery b;
  while (1) {
    bms.getData(&b);
    log.INFO("TEST", "volatage: %d, temp: %d, current: %d, charge: %d", 
      b.voltage,
      b.temperature,
      b.current,
      b.charge);
    Thread::sleep(100);
  }

  return 0;
}
