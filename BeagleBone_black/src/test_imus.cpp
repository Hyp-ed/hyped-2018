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


#include "sensors/mpu9250.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"
#include "utils/timer.hpp"
#include "sensors/interface.hpp"
#include "sensors/fake_imu.hpp"

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using hyped::data::Imu;
using hyped::sensors::ImuInterface;
using hyped::sensors::FakeImu;

#include <string>
#include <iostream>
#include <fstream>



int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);
  uint8_t chip_select_[] = {48, 49, 117, 115};
  ImuInterface*   imu_[hyped::data::Sensors::kNumImus];
  std::ofstream myfile[hyped::data::Sensors::kNumImus];

  for (int i = 0; i < hyped::data::Sensors::kNumImus; i++) {
    myfile[i].open("imu_" + std::to_string(i) + "_test.csv");
  }

  for (int i = 0; i < hyped::data::Sensors::kNumImus; i++) {
    myfile[i] << "Timestamp µs, acc x, acc y, acc z, gyro x, gyro y, gyro z, operational\n";
  }
  

  for (int i = 0; i < hyped::data::Sensors::kNumImus; i++) {
    imu_[i] = new MPU9250(log, chip_select_[i], 0x08, 0x00);
  }
  log.INFO("TEST-mpu9260", "MPU9250 instance successfully created");

  uint64_t start = hyped::utils::Timer::getTimeMicros();
  while (120000 > (hyped::utils::Timer::getTimeMicros() - start)) {
    for (int j = 0; j < hyped::data::Sensors::kNumImus; j ++) {
      hyped::data::Imu imu;
      imu_[j]->getData(&imu);
      myfile[j] << std::to_string(hyped::utils::Timer::getTimeMicros() - start) << ",";
      myfile[j] << std::to_string(imu.acc[0]) << ",";
      myfile[j] << std::to_string(imu.acc[1]) << ",";
      myfile[j] << std::to_string(imu.acc[2]) << ",";
      myfile[j] << std::to_string(imu.gyr[0]) << ",";
      myfile[j] << std::to_string(imu.gyr[1]) << ",";
      myfile[j] << std::to_string(imu.gyr[2]) << ",";
      if (imu.operational) {
        myfile[j] << "true" << "\n";
      } else {
        myfile[j] << "false" << "\n";
      }
   }
   Thread::sleep(50);
  }
  for (int i = 0; i < hyped::data::Sensors::kNumImus; i++) {
    myfile[i].close();
  }
 	return 0;
}
