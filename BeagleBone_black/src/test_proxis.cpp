/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 29/06/18
 * Description: Demo for the multiplexer with proxisensors
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
#include "sensors/interface.hpp"
#include "utils/math/statistics.hpp"
#include "utils/timer.hpp"

#include <iostream>
#include <fstream>

using hyped::sensors::VL6180;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using hyped::utils::io::I2C;
using hyped::sensors::ProxiInterface;
using hyped::utils::math::RollingStatistics;
using hyped::utils::Timer;

 constexpr uint8_t kNumOfProxis = 8;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  I2C& i2c = I2C::getInstance();
  Logger log(true, 1);
  log.INFO("TEST-vl6180", "VL6180 instance successfully created");
  uint8_t kMultiplexerAddr = 0x70;
  VL6180* proxi_[kNumOfProxis];
  std::ofstream myfile;
  myfile.open ("proxi_test.csv");
  myfile << "Timestamp µs, Sensor 1, Sensor 2, Sensor 3, Sensor 4, Sensor 5, Sensor 6, Sensor 7, Sensor 8\n";

  for (int i = 0; i < kNumOfProxis; i++) {
      i2c.write(kMultiplexerAddr, 0x01 << i);  // open particular i2c channel
      log.INFO("Multiplexer", "Opening channel: %d", i);
      VL6180* proxi = new VL6180(0x29, log);
      proxi_[i] = proxi;
    }

  uint64_t start = hyped::utils::Timer::getTimeMicros();
    while (120000000 > (hyped::utils::Timer::getTimeMicros() - start)) {
    myfile << hyped::utils::Timer::getTimeMicros() - start;
    for (int j = 0; j < kNumOfProxis; j++) {
      i2c.write(kMultiplexerAddr, 0x01 << j);  // open particular i2c channel
      hyped::data::Proximity proxi;
      proxi_[j]->getData(&proxi);
      myfile << proxi.val << ",";
      // log.INFO("Multiplexer-test", "Sensor %d, reading %d", j, proxi.val);
      // log.INFO("Multiplexer-test", "operational: %s", proxi.operational ? "true" : "false");
    }
    myfile << "\n";
    Thread::sleep(10);
  }

  myfile.close();

  return 0;
}