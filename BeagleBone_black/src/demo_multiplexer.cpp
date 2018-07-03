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

using hyped::sensors::VL6180;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using hyped::utils::io::I2C;
using hyped::sensors::ProxiInterface;

Logger log(true, 1);

// 
    
//     i2c.write(kMultiplexerAddr, 0xFF);      // open all i2c channels
int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  I2C& i2c = I2C::getInstance();
  log.INFO("TEST-vl6180", "VL6180 instance successfully created");
  uint8_t kMultiplexerAddr = 0x70;
  uint8_t kNumOfProxis = 2;
  ProxiInterface* proxi_[hyped::data::Sensors::kNumProximities];

  for (int i = 0; i < kNumOfProxis; i++) {
      i2c.write(kMultiplexerAddr, 1 << i);  // open particular i2c channel
      VL6180* proxi = new VL6180(0x29, log);
      proxi->setContinuousRangingMode();
      proxi->setAddress(0x29 + i);
      proxi_[i] = proxi;
    }

  for (int i = 0; i < 300; i++) {
    // update front cluster of proximities
    for (int i = 0; i < kNumOfProxis; i++) {
      hyped::data::Proximity proxi;
      proxi_[i%kNumOfProxis]->getData(&proxi);
      log.INFO("Multiplexer-test", "Sensor %d, reading %d", i%kNumOfProxis, proxi.val);
      Thread::sleep(3);
    }

  }
  Thread::sleep(10);

 	return 0;
}