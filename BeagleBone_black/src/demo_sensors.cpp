/*
 * Author: Martin Kristien, Jack Horsburgh and Ragnor Comerford
 * Organisation: HYPED
 * Date: 13/03/18
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

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "data/data.hpp"
#include "sensors/main.hpp"
#include "communications/main.hpp"
#include "utils/system.hpp"
#include "utils/logger.hpp"
using namespace hyped;
using data::Data;
using data::Sensors;
using data::Batteries;
using data::StripeCounter;
using data::SensorCalibration;
using utils::System;
using utils::Logger;

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System& sys = System::getSystem();
  Logger& log = utils::System::getLogger();
  Data& data = Data::getInstance();

  Thread* sensors   = new sensors::Main(2, log);
  Thread* communications = new communications::Main(4, log);
  sensors->start();
  communications->start();

  Sensors sens;
  Batteries batteries;
  while (sys.running_) {
    sens = data.getSensorsData();
    batteries = data.getBatteriesData();

    log.INFO("Test", "Low Power Batteries");
    for (auto& battery : batteries.low_power_batteries) {
      log.INFO("TEST", "Charge: %u, Temp: %d, Voltage: %u, Current %d", battery.charge,
                                                                        battery.temperature,
                                                                        battery.voltage,
                                                                        battery.current);
    }

    std::printf("\n");
    log.INFO("Test", "High Power Batteries");
    for (auto& battery : batteries.high_power_batteries) {
      log.INFO("TEST", "Charge: %u, Temp: %d, Voltage: %u, Current %d, Min: %u, Max: %u", battery.charge,
                                                                        battery.temperature,
                                                                        battery.voltage,
                                                                        battery.current,
                                                                        battery.low_voltage_cell,
                                                                        battery.high_voltage_cell);
    }

    std::printf("\n");
    auto imus = sens.imu.value;
    log.INFO("Test", "IMUs online: %d,%d,%d,%d", imus[0].operational,
                                                 imus[1].operational,
                                                 imus[2].operational,
                                                 imus[3].operational);
    for (auto& imu_data : sens.imu.value) {
      auto& acc = imu_data.acc;
      auto& gyr = imu_data.gyr;
      log.INFO("TEST", "Acceleration  (% 2.4f % 2.4f % 2.4f)  Gyroscope  (% 2.4f % 2.4f % 2.4f)",
        acc[0],
        acc[1],
        acc[2],
        gyr[0],
        gyr[1],
        gyr[2]);
    }

    std::printf("\n");
    auto proxis_front = sens.proxi_front.value;
    log.INFO("Test", "Proxi-front online: %d,%d,%d,%d,%d,%d,%d,%d", proxis_front[0].operational,
                                                             proxis_front[1].operational,
                                                             proxis_front[2].operational,
                                                             proxis_front[3].operational,
                                                             proxis_front[4].operational,
                                                             proxis_front[5].operational,
                                                             proxis_front[6].operational,
                                                             proxis_front[7].operational);
    for (auto& proxi_data : sens.proxi_front.value) {
      auto proxi = proxi_data.val;
      log.INFO("TEST", "Distance front:  %u", proxi);
    }

    std::printf("\n");
    auto proxis_back = sens.proxi_back.value;
    log.INFO("Test", "Proxi-back online: %d,%d,%d,%d,%d,%d,%d,%d", proxis_back[0].operational,
                                                             proxis_back[1].operational,
                                                             proxis_back[2].operational,
                                                             proxis_back[3].operational,
                                                             proxis_back[4].operational,
                                                             proxis_back[5].operational,
                                                             proxis_back[6].operational,
                                                             proxis_back[7].operational);
    for (auto& proxi_data : sens.proxi_back.value) {
      auto proxi = proxi_data.val;
      log.INFO("TEST", "Distance back: %u", proxi);
    }
    Thread::sleep(500);
  }
 	return 0;
}
