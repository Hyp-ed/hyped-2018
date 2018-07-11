/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 10/07/18
 * Description: Demo for multiple IMU sensors
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

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using hyped::data::Imu;



int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);
  MPU9250 imu1 = MPU9250(log, 51, 0x08, 0x00);
  MPU9250 imu2 = MPU9250(log, 48, 0x08, 0x00);
  MPU9250 imu3 = MPU9250(log, 40, 0x08, 0x00);
  MPU9250 imu4 = MPU9250(log, 31, 0x08, 0x00);

  Imu imu;

  log.INFO("DEMO", "IMU1");

  for (int i=0; i< 10; i++) {
    imu1.getData(&imu);
    log.DBG("DEMO", "accelerometer readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2", imu.acc[0], imu.acc[1], imu.acc[2]);
    // log.DBG("DEMO", "gyroscope readings     x: %f rad/s, y: %f rad/s, z: %f rad/s", imu1.gyr[0], imu1.gyr[1], imu1.gyr[2]);
    Thread::sleep(10);
  }

  log.INFO("DEMO", "IMU2");

  for (int i=0; i< 10; i++) {
    imu2.getData(&imu);
    log.DBG("DEMO", "accelerometer readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2", imu.acc[0], imu.acc[1], imu.acc[2]);
    // log.DBG("DEMO", "gyroscope readings     x: %f rad/s, y: %f rad/s, z: %f rad/s", imu1.gyr[0], imu1.gyr[1], imu1.gyr[2]);
    Thread::sleep(10);
  }

  log.INFO("DEMO", "IMU3");

  for (int i=0; i< 10; i++) {
    imu3.getData(&imu);
    log.DBG("DEMO", "accelerometer readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2", imu.acc[0], imu.acc[1], imu.acc[2]);
    // log.DBG("DEMO", "gyroscope readings     x: %f rad/s, y: %f rad/s, z: %f rad/s", imu1.gyr[0], imu1.gyr[1], imu1.gyr[2]);
    Thread::sleep(10);
  }

  log.INFO("DEMO", "IMU4");

  for (int i=0; i< 10; i++) {
    imu4.getData(&imu);
    log.DBG("DEMO", "accelerometer readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2", imu.acc[0], imu.acc[1], imu.acc[2]);
    // log.DBG("DEMO", "gyroscope readings     x: %f rad/s, y: %f rad/s, z: %f rad/s", imu1.gyr[0], imu1.gyr[1], imu1.gyr[2]);
    Thread::sleep(10);
  }

 	return 0;
}