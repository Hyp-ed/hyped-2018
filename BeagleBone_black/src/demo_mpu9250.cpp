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

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;



int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 1);
  MPU9250 mpu9250 = MPU9250(log, 60, 0x08, 0x00);

  log.INFO("TEST-mpu9260", "MPU9250 instance successfully created");

  for (int i=0; i< 100; i++) {
    mpu9250.getAcclData();
    log.DBG("TEST-mpu9250", "accelerometer readings x: %f", mpu9250.accel_data_[0]);
    log.DBG("TEST-mpu9250", "accelerometer readings y: %f", mpu9250.accel_data_[1]);
    log.DBG("TEST-mpu9250", "accelerometer readings z: %f", mpu9250.accel_data_[2]);
    Thread::sleep(50);
  }

  for (int i=0; i< 50; i++) {
    mpu9250.getGyroData();
    log.DBG("TEST-mpu9250", "gyroscope readings x: %f", mpu9250.gyro_data_[0]);
    log.DBG("TEST-mpu9250", "gyroscope readings y: %f", mpu9250.gyro_data_[1]);
    log.DBG("TEST-mpu9250", "gyroscope readings z: %f", mpu9250.gyro_data_[2]);
    Thread::sleep(50);
  }


 	return 0;
}
