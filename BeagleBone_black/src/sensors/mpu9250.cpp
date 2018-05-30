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
#include <chrono>
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
constexpr uint8_t  GYRO_XOUT_H           = 0x43;
constexpr uint8_t  GYRO_XOUT_L           = 0x44;
constexpr uint8_t  GYRO_YOUT_H           = 0x45;
constexpr uint8_t  GYRO_YOUT_L           = 0x46;
constexpr uint8_t  GYRO_ZOUT_H           = 0x47;
constexpr uint8_t  GYRO_ZOUT_L           = 0x48;

constexpr uint8_t  GYRO_CONFIG           = 0x1B;

constexpr uint8_t WHO_AM_I_MPU9250       = 0x75;
constexpr uint8_t WHO_AM_I_RESET_VALUE   = 0x71;

// Power Management
constexpr uint8_t REG_PWR_MGMT_1         = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2         = 0x6C;

constexpr uint8_t kReadFlag              = 0x80;



// Configuration bits mpu9250
// Resets the device to defaults
#define BIT_H_RESET 0x80

namespace hyped {

utils::io::gpio::Direction kDirection = utils::io::gpio::kOut;

namespace sensors {

const uint64_t MPU9250::time_start = utils::Timer::getTimeMicros();

MPU9250::MPU9250(Logger& log, uint32_t pin)
    :gpio_(pin, kDirection, log),
    log_(log)
{
  init();
  log_.INFO("MPU9250", "Creating a sensor with id: %d", 1);
}

void MPU9250::init()
{
  // Set pin high
  gpio_.set();

  // TODO(anyone) Check who am I
  // Wil stay in while look as it is not connected properly
  while (!whoAmI());

  // Reset device -
  writeByte(REG_PWR_MGMT_1, BIT_H_RESET);
  // Wait until completed

  // Set clock source
  writeByte(REG_PWR_MGMT_1, 0x01);

  // Enable accelerometer and gyroscope
  writeByte(REG_PWR_MGMT_2, 0x00);

  // Gyroscope and accelerometer calibration and set scales
}

bool MPU9250::whoAmI()
{
  uint8_t data;

  // Who am I checks what address the sensor is at
  readByte(WHO_AM_I_MPU9250, &data);

  // TODO(anyone) need to find what it should be equal to
  if (data != WHO_AM_I_RESET_VALUE) {
     log_.ERR("MPU9250", "Cannot initialise who am I is incorrect");
    return false;
  }

  return true;
}


MPU9250::~MPU9250()
{
  log_.INFO("MPU9250", "Deconstructing sensor object");
}

void MPU9250::writeByte(uint8_t write_reg, uint8_t write_data)
{
  select();
  spi_.write(write_reg, &write_data, 1);
  deSelect();
}

void MPU9250::readByte(uint8_t read_reg, uint8_t *read_data)
{
  select();
  spi_.read(read_reg, read_data, 1);
  deSelect();
}

void MPU9250::readBytes(uint8_t read_reg, uint8_t *read_data, uint8_t length)
{
  int i;
  for (i = 0; i < length; i++) {
    readByte(read_reg + i, &read_data[i]);
  }
}

void MPU9250::performSensorReadings()
{/*EMPTY*/}

double MPU9250::getAcclData()
{
  uint8_t response[6];
  int16_t bit_data;
  double data;
  int i;

  readBytes(ACCEL_XOUT_H, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    data = static_cast<double>(bit_data);
    // TODO(anyone) need to look back at here
    accel_data_[i] = data;
  }
}

double MPU9250::getGyroData()
{/*EMPTY*/}

void MPU9250::setGyroScale(int scale)
{/*EMPTY*/}

void MPU9250::setAcclScale(int scale)
{/*EMPTY*/}

void MPU9250::calibrateAccl()
{/*EMPTY*/}

void MPU9250::select()
{
  gpio_.clear();
}
void MPU9250::deSelect()
{
  gpio_.set();
}

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
