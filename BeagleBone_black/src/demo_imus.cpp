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

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using hyped::data::Imu;
using hyped::sensors::ImuInterface;




int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);
  uint8_t chip_select_[] = {48, 49, 117, 115};
  ImuInterface*   imu_[hyped::data::Sensors::kNumImus];

  for (int i = 0; i < hyped::data::Sensors::kNumImus; i++) {
      imu_[i] = new MPU9250(log, chip_select_[i], 0x08, 0x00);
  }
  log.INFO("TEST-mpu9260", "MPU9250 instance successfully created");

  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < hyped::data::Sensors::kNumImus; j ++) {
        hyped::data::Imu imu;
        imu_[j]->getData(&imu);
        log.DBG("TEST-mpu9250", "accelerometer id: %d,  x: %f m/s^2, y: %f m/s^2, z: %f m/s^2",
                j,
                imu.acc[0],
                imu.acc[1],
                imu.acc[2]);
        log.DBG("TEST-mpu9250", "gyroscope id: %d,  x: %f rad/s, y: %f rad/s, z: %f rad/s",
                j,
                imu.gyr[0],
                imu.gyr[1],
                imu.gyr[2]);
    }
    Thread::sleep(500);
  }
 	return 0;
}
