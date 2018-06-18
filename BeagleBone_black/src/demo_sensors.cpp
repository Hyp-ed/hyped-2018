/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 18. June 2018
 * Description:
 * Drive connected sensors, take and log all readings.
 * Sensors supported: Proxi, IMU, BMS LP and HP, SAM CAN/PRoxi
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

#include "data/data.hpp"

#include "sensors/bms.hpp"
#include "sensors/mpu9250.hpp"
#include "sensors/vl6180.hpp"
#include "sensors/can_proxi.hpp"

#include "utils/system.hpp"
#include "utils/logger.hpp"

constexpr uint8_t BMS_LP = 2;
constexpr uint8_t BMS_HP = 1;

using hyped::data::Sensors;
using hyped::data::Batteries;

using hyped::sensors::ProxiInterface;

using hyped::utils::Logger;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();
  log.INFO("MAIN", "system started, logger created");

  log.INFO("MAIN", "creating sensors");
  log.INFO("MAIN", "creating VL6180");
  ProxiInterface* proxi_here = new hyped::sensors::VL6180(0x29);

  Sensors   sensors;
  Batteries batteries;
  log.INFO("MAIN", "all sensors created, entering test loop");
  while (1) {
    proxi_here->getData(&sensors.proxi_front[0]);
    log.INFO("TEST", "proxi here distance: %u",
      sensors.proxi_front[0].val);
  }
}


