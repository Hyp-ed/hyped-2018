/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 23/05/18
 * Description: Main file for MPU9250
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
#include <cstdint>

#include "sensors/mpu9250.hpp"
#include "utils/logger.hpp"
#include "utils/timer.hpp"

// Accelerometer addresses
constexpr uint8_t ACCEL_XOUT_H       = 0x3B;
constexpr uint8_t ACCEL_XOUT_L       = 0x3C;
constexpr uint8_t ACCEL_YOUT_H       = 0x3D;
constexpr uint8_t ACCEL_YOUT_L       = 0x3E;
constexpr uint8_t ACCEL_ZOUT_H       = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L       = 0x40;
constexpr uint8_t ACCEL_CONFIG       = 0x1C;
constexpr uint8_t ACCEL_CONFIG2      = 0x1D;

// gyroscope addresses
constexpr uint8_t  GYRO_XOUT_H        = 0x43;
constexpr uint8_t  GYRO_XOUT_L        = 0x44;
constexpr uint8_t  GYRO_YOUT_H        = 0x45;
constexpr uint8_t  GYRO_YOUT_L        = 0x46;
constexpr uint8_t  GYRO_ZOUT_H        = 0x47;
constexpr uint8_t  GYRO_ZOUT_L        = 0x48;
constexpr uint8_t  GYRO_CONFIG        = 0x1B;

constexpr uint8_t WHO_AM_I_MPU9250    = 0x75;

namespace hyped {
namespace sensors {

const uint64_t MPU9250::time_start = utils::Timer::getTimeMicros();

MPU9250::MPU9250(Logger& log)
    : log_(log)
{
  init();
  log_.INFO("MPU9250", "Creating a sensor with id: %d", 1);
}

void MPU9250::init()
{/*EMPTY*/}


MPU9250::~MPU9250()
{
  log_.INFO("MPU9250", "Deconstructing sensor object");
}



bool MPU9250::readByte(uint8_t read_reg, uint8_t *read_data)
{
  // select
  spi_.read(read_reg, read_data, 1);
  // deselect

  return false;
}

void MPU9250::performSensorReadings()
{/*EMPTY*/}

double MPU9250::getAcclData()
{/*EMPTY*/}

double MPU9250::getGyroData()
{/*EMPTY*/}

void MPU9250::setGyroScale(int scale)
{/*EMPTY*/}

void MPU9250::setAcclScale(int scale)
{/*EMPTY*/}

void MPU9250::calibrateAccl()
{/*EMPTY*/}

void MPU9250::getData(Imu* imu)
{
  auto acc = imu->acc.value;
  auto gyr = imu->gyr.value;
  acc[0] = 1.0;
  acc[1] = 0.0;
  acc[2] = 0.0;
  gyr = {};

  uint32_t time = utils::Timer::getTimeMillis() - (time_start/1000);
  imu->acc.timestamp = time;
  imu->gyr.timestamp = time;
}

}}   // namespace hyped::sensors
