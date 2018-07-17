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
#include "utils/concurrent/thread.hpp"
#include "utils/io/i2c.hpp"
#include "data/data.hpp"

using hyped::sensors::VL6180;
using hyped::utils::Logger;
using hyped::utils::io::I2C;
using hyped::utils::concurrent::Thread;


int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  I2C& i2c = I2C::getInstance();
  Logger log(true, 1);
  i2c.write(0x70, 0x01);
  VL6180 proxi = VL6180(0x29, log);

  log.INFO("TEST-vl6180", "VL6180 instance successfully created");

  for (int i=0; i< 500; i++) {
    hyped::data::Proximity data;
    proxi.getData(&data);
    log.INFO("TEST-vl6180", "Continuous Distance: %d", data.val);
    log.INFO("Multiplexer-test", "operational: %s", data.operational ? "true" : "false");
    Thread::sleep(10);
  }



 	return 0;
}
