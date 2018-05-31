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

#include <chrono>
#include <cstdint>
#include <unistd.h>

#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"
#include "utils/timer.hpp"

// Accelerometer addresses
constexpr uint8_t ACCEL_XOUT_H           = 0x3B;
constexpr uint8_t ACCEL_XOUT_L           = 0x3C;
constexpr uint8_t ACCEL_YOUT_H           = 0x3D;
constexpr uint8_t ACCEL_YOUT_L           = 0x3E;
constexpr uint8_t ACCEL_ZOUT_H           = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L           = 0x40;

constexpr uint8_t ACCEL_CONFIG           = 0x1C;
constexpr uint8_t ACCEL_CONFIG2          = 0x1D;

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

// User Control
constexpr uint8_t MPU9250_REG_USER_CTRL  = 0x6A;

// I2C Master Control
constexpr uint8_t MPU9250_REG_I2C_MST_CTRL  = 0x24;

// Power Management
constexpr uint8_t MPU9250_REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t MPU9250_REG_PWR_MGMT_2    = 0x6C;

// Configuration
#define MPU9250_REG_CONFIG              0x1A

// Sample Rate Divider
#define MPU9250_REG_SMPLRT_DIV          0x19

#define MPUREG_FIFO_COUNTH 0x72

// FIFO Enable
constexpr uint8_t MPU9250_REG_FIFO_EN       = 0x23;

// Interrupt Enable
constexpr uint8_t MPU9250_REG_INT_ENABLE    = 0x38;

constexpr uint8_t kReadFlag              = 0x80;

// Configuration bits mpu9250
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18




// Configuration bits mpu9250
// Resets the device to defaults
#define BIT_H_RESET 0x80

namespace hyped {

utils::io::gpio::Direction kDirection = utils::io::gpio::kOut;
using utils::concurrent::Thread;

namespace sensors {

const uint64_t MPU9250::time_start = utils::Timer::getTimeMicros();

MPU9250::MPU9250(Logger& log, uint32_t pin, bool isSpi, uint8_t i2c_addr)
    :log_(log),
    isSpi_(isSpi),
    gpio_(pin, kDirection, log),
    i2c_addr_(i2c_addr)
{
  init();
  log_.INFO("MPU9250", "Creating a sensor with id: %d", 1);
}

void MPU9250::init()
{
  // Set pin high
  gpio_.set();
  Thread::sleep(100);   // 100ms

  writeByte(MPU9250_REG_PWR_MGMT_1, BIT_H_RESET);   // Reset Device
  Thread::sleep(100);    // 100ms
  writeByte(MPU9250_REG_USER_CTRL, 0x20);   // set I2C_IF_DIS to disable slave mode I2C bus
  Thread::sleep(1000);    // 100ms

  // writeByte(MPU9250_REG_PWR_MGMT_1, 0x01);          // Clock Source
  // writeByte(MPU9250_REG_PWR_MGMT_2, 0x00);          // Enable Acc & Gyro
  // writeByte(MPU9250_REG_CONFIG, 0x01);
  // writeByte(GYRO_CONFIG, 0x00);
  // writeByte(ACCEL_CONFIG, 0x00);
  // writeByte(ACCEL_CONFIG2, 0x01);

  // TODO(anyone) Check who am I
  // Will stay in while look as it is not connected properly
  while (!whoAmI());

  // Enable accelerometer and gyroscope
  // writeByte(MPU9250_REG_PWR_MGMT_2, 0x00);
}

void MPU9250::calibrateSensors()
{
  // TODO(jack) finish calibration

  uint8_t data[12];   // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;

  // Configure device for bias calculation
  // Disable i2c if neccessary
  writeByte(MPU9250_REG_INT_ENABLE, 0x00);    // Disable all interrupts
  writeByte(MPU9250_REG_FIFO_EN, 0x00);       // Disable FIFO
  writeByte(MPU9250_REG_PWR_MGMT_1, 0x00);    // Turn on internal clock source
  writeByte(MPU9250_REG_I2C_MST_CTRL, 0x00);  // Disable I2C master
  writeByte(MPU9250_REG_USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
  writeByte(MPU9250_REG_USER_CTRL, 0x0C);     // Reset FIFO and DMP
  // Wait for 15ms

  // TODO(anyone) Configure gyro and accelerometer
  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_REG_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  writeByte(MPU9250_REG_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(GYRO_CONFIG, 0x00);
  // Wait 40ms until completed

  writeByte(ACCEL_CONFIG, 0x00);   // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;     // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;   // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_REG_USER_CTRL, 0x40);    // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  writeByte(MPU9250_REG_FIFO_EN, 0x78);
  // Wait 40ms Accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_REG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPUREG_FIFO_COUNTH, data, 2);    // read FIFO sample count
  fifo_count = ( (uint16_t) data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;
}

bool MPU9250::whoAmI()
{
  uint8_t data;

  // Who am I checks what address the sensor is at
  readByte(WHO_AM_I_MPU9250, &data);
  log_.ERR("MPU9250", "who am I: %u", data);
  // TODO(anyone) need to find what it should be equal to
  if (data != WHO_AM_I_RESET_VALUE) {
    Thread::sleep(1000);
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
  if (isSpi_) {
    // Write byte for spi
    select();
    spi_.write(write_reg, &write_data, 1);
    deSelect();
  } else {
    // Write byte for i2c
    uint8_t buffer[2];
    buffer[0]=write_reg;
    buffer[1]=write_data;
    i2c_.write(i2c_addr_, buffer, 2);
  }
}

void MPU9250::readByte(uint8_t read_reg, uint8_t *read_data)
{
  if (isSpi_) {
    // Read byte for spi
    select();
    spi_.read(read_reg | kReadFlag, read_data, 1);
    deSelect();
  } else {
    // Read byte for i2c
    i2c_.write(i2c_addr_, &read_reg, 1);
    i2c_.read(i2c_addr_, read_data, 1);
  }
}

// TODO(jack) put into one read as a buffer
void MPU9250::readBytes(uint8_t read_reg, uint8_t *read_data, uint8_t length)
{
  int i;
  for (i = 0; i < length; i++) {
    readByte(read_reg + i, &read_data[i]);
  }
}

void MPU9250::getAcclData()
{
  uint8_t response[6];
  int16_t bit_data;
  double data;
  int i;

  readBytes(ACCEL_XOUT_H, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    // TODO(anyone) change casting
    data = static_cast<int>(bit_data);
    // TODO(anyone) need to look back at here when scale added
    accel_data_[i] = data/acc_divider_;
  }
}

void MPU9250::getGyroData()
{
  uint8_t response[6];
  int16_t bit_data;
  double data;
  int i;

  readBytes(GYRO_XOUT_H, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    // TODO(anyone) change casting
    data = static_cast<int>(bit_data);
    // TODO(anyone) need to look back at here when scale added
    gyro_data_[i] = data/gyro_divider_;
  }
}

void MPU9250::setGyroScale(int scale)
{
  writeByte(GYRO_CONFIG, scale);

  switch (scale) {
    case BITS_FS_250DPS:
      gyro_divider_ = 131;
    break;
    case BITS_FS_500DPS:
      gyro_divider_ = 65.5;
      break;
    case BITS_FS_1000DPS:
      gyro_divider_ = 32.8;
    break;
    case BITS_FS_2000DPS:
      gyro_divider_ = 16.4;
    break;
  }
}

void MPU9250::setAcclScale(int scale)
{
  writeByte(ACCEL_CONFIG, scale);

  switch (scale) {
    case BITS_FS_2G:
      acc_divider_ = 16384;
    break;
    case BITS_FS_4G:
      acc_divider_ = 8192;
    break;
    case BITS_FS_8G:
      acc_divider_ = 4096;
    break;
    case BITS_FS_16G:
      acc_divider_ = 2048;
    break;
  }
}

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
