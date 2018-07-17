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
#include "sensors/mpu9250.hpp"

#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"
#include "utils/math/statistics.hpp"

// Accelerometer addresses
constexpr uint8_t kAccelXoutH               = 0x3B;

constexpr uint8_t kAccelConfig              = 0x1C;
constexpr uint8_t kAccelConfig2             = 0x1D;

constexpr uint8_t  kGyroConfig              = 0x1B;

constexpr uint8_t kWhoAmIMpu9250            = 0x75;
constexpr uint8_t kWhoAmIResetValue1        = 0x71;
constexpr uint8_t kWhoAmIResetValue2        = 0x73;

// Power Management
constexpr uint8_t kMpuRegPwrMgmt1           = 0x6B;

// Configuration
constexpr uint8_t kMpuRegConfig             = 0x1A;

constexpr uint8_t kReadFlag                 = 0x80;

// Configuration bits mpu9250
constexpr uint8_t kBitsFs250Dps             = 0x00;
constexpr uint8_t kBitsFs500Dps             = 0x08;
constexpr uint8_t kBitsFs1000Dps            = 0x10;
constexpr uint8_t kBitsFs2000Dps            = 0x18;
constexpr uint8_t kBitsFs2G                 = 0x00;
constexpr uint8_t kBitsFs4G                 = 0x08;
constexpr uint8_t kBitsFs8G                 = 0x10;
constexpr uint8_t kBitsFs16G                = 0x18;

// Resets the device to defaults
constexpr uint8_t kBitHReset                = 0x80;

namespace hyped {

utils::io::gpio::Direction kDirection = utils::io::gpio::kOut;
using utils::concurrent::Thread;
using utils::math::OnlineStatistics;
using data::NavigationVector;

namespace sensors {

MPU9250::MPU9250(Logger& log, uint32_t pin, uint8_t acc_scale, uint8_t gyro_scale)
    : log_(log),
    gpio_(pin, kDirection, log),
    acc_scale_(acc_scale),
    gyro_scale_(gyro_scale),
    is_online_(false)
{
  init();
  log_.DBG("MPU9250", "Creating IMU sensor");
}

void MPU9250::init()
{
  // Set pin high
  gpio_.set();

  writeByte(kMpuRegPwrMgmt1, kBitHReset);   // Reset Device
  Thread::sleep(200);
  // Test connection
  whoAmI();

  writeByte(kMpuRegConfig, 0x01);
  writeByte(kAccelConfig2, 0x01);
  setAcclScale(acc_scale_);
  setGyroScale(gyro_scale_);
  log_.INFO("MPU9250", "IMU sensor created. Initialisation complete");
}

bool MPU9250::whoAmI()
{
  uint8_t data;
  int send_counter;

  for (send_counter = 0; send_counter < 10; send_counter++) {
    // Who am I checks what address the sensor is at
    readByte(kWhoAmIMpu9250, &data);
    if (data == kWhoAmIResetValue1 || data == kWhoAmIResetValue2) {
      log_.INFO("MPU9250", "IMU connected to SPI");
      is_online_ = true;
      break;
    } else {
      log_.DBG1("MPU9250", "Cannot initialise. Who am I is incorrect");
      is_online_ = false;
      Thread::yield();
    }
  }

  if (!is_online_) {
    log_.ERR("MPU9250", "Cannot initialise who am I. Sensor offline");
  }
  return is_online_;
}

MPU9250::~MPU9250()
{
  log_.INFO("MPU9250", "Deconstructing sensor object");
}

void MPU9250::writeByte(uint8_t write_reg, uint8_t write_data)
{
  // ',' instead of ';' is to inform the compiler not to reorder function calls
  // chip selects signals must have exact ordering with respect to the spi access
  select(),
  spi_.write(write_reg, &write_data, 1),
  deSelect();
}

void MPU9250::readByte(uint8_t read_reg, uint8_t *read_data)
{
  select(),
  spi_.read(read_reg | kReadFlag, read_data, 1),
  deSelect();
}

void MPU9250::readBytes(uint8_t read_reg, uint8_t *read_data, uint8_t length)
{
  select(),
  spi_.read(read_reg | kReadFlag, read_data, length),
  deSelect();
}

void MPU9250::select()
{
  gpio_.clear();
}
void  MPU9250::deSelect()
{
  gpio_.set();
}

void MPU9250::setGyroScale(int scale)
{
  writeByte(kGyroConfig, scale);

  switch (scale) {
    case kBitsFs250Dps:
      gyro_divider_ = 131;
    break;
    case kBitsFs500Dps:
      gyro_divider_ = 65.5;
      break;
    case kBitsFs1000Dps:
      gyro_divider_ = 32.8;
    break;
    case kBitsFs2000Dps:
      gyro_divider_ = 16.4;
    break;
  }
}

void MPU9250::setAcclScale(int scale)
{
  writeByte(kAccelConfig, scale);

  switch (scale) {
    case kBitsFs2G:
      acc_divider_ = 16384;
    break;
    case kBitsFs4G:
      acc_divider_ = 8192;
    break;
    case kBitsFs8G:
      acc_divider_ = 4096;
    break;
    case kBitsFs16G:
      acc_divider_ = 2048;
    break;
  }
}

void MPU9250::getData(Imu* imu)
{
  if (is_online_) {
    log_.DBG3("MPU9250", "Getting IMU data");
    auto& acc = imu->acc;
    auto& gyr = imu->gyr;
    uint8_t response[14];
    int16_t bit_data;
    float data;
    int i;
    float accel_data[3];
    float gyro_data[3];

    readBytes(kAccelXoutH, response, 14);
    for (i = 0; i < 3; i++) {
      bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
      data = static_cast<float>(bit_data);
      accel_data[i] = data/acc_divider_  * 9.80665;

      bit_data = ((int16_t) response[i*2 + 8] << 8) | response[i*2+9];
      data = static_cast<float>(bit_data);
      gyro_data[i] = data/gyro_divider_;
    }
    imu->operational = is_online_;
    acc[0] = accel_data[0];
    acc[1] = accel_data[1];
    acc[2] = accel_data[2];
    gyr[0] = gyro_data[0];
    gyr[1] = gyro_data[1];
    gyr[2] = gyro_data[2];
  } else {
    // Try and turn the sensor on again
    log_.ERR("MPU9250", "Sensor not operational, trying to turn on sensor");
    init();
  }
}

}}   // namespace hyped::sensors
