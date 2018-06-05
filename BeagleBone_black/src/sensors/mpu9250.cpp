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
constexpr uint8_t kAccelXoutH           = 0x3B;

#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E

constexpr uint8_t ACCEL_CONFIG           = 0x1C;
constexpr uint8_t ACCEL_CONFIG2          = 0x1D;

// gyroscope addresses
constexpr uint8_t  kGyroXoutH           = 0x43;

#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18

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
#define MPUREG_FIFO_R_W 0x74

// Resets the device to defaults
#define BIT_H_RESET 0x80

namespace hyped {

utils::io::gpio::Direction kDirection = utils::io::gpio::kOut;
using utils::concurrent::Thread;

namespace sensors {

const uint64_t MPU9250::time_start = utils::Timer::getTimeMicros();

MPU9250::MPU9250(Logger& log, uint32_t pin, bool isSpi, uint8_t i2c_addr)
    : log_(log),
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
  calibrateSensors();

  // Test connection
  while (!whoAmI());

  writeByte(MPU9250_REG_PWR_MGMT_1, BIT_H_RESET);   // Reset Device
  // writeByte(MPU9250_REG_USER_CTRL, 0x20);   // set I2C_IF_DIS to disable slave mode I2C bus
  writeByte(MPU9250_REG_PWR_MGMT_1, 0x01);          // Clock Source
  writeByte(MPU9250_REG_PWR_MGMT_2, 0x00);          // Enable Acc & Gyro
  writeByte(MPU9250_REG_CONFIG, 0x01);
  writeByte(GYRO_CONFIG, 0x00);
  writeByte(ACCEL_CONFIG, 0x00);
  writeByte(ACCEL_CONFIG2, 0x01);
  setAcclScale(BITS_FS_2G);
  setGyroScale(BITS_FS_250DPS);
}

void MPU9250::calibrateSensors()
{
  uint8_t data[12];
  uint16_t ii, packet_count, fifo_count;

  // reset device
  writeByte(MPU9250_REG_PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
  Thread::sleep(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_REG_PWR_MGMT_1, 0x01);
  writeByte(MPU9250_REG_PWR_MGMT_2, 0x00);
  Thread::sleep(200);

  // Configure device for bias calculation
  writeByte(MPU9250_REG_INT_ENABLE, 0x00);    // Disable all interrupts
  writeByte(MPU9250_REG_FIFO_EN, 0x00);       // Disable FIFO
  writeByte(MPU9250_REG_PWR_MGMT_1, 0x00);    // Turn on internal clock source
  writeByte(MPU9250_REG_I2C_MST_CTRL, 0x00);  // Disable I2C master
  writeByte(MPU9250_REG_USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
  writeByte(MPU9250_REG_USER_CTRL, 0x0C);     // Reset FIFO and DMP
  Thread::sleep(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_REG_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  writeByte(MPU9250_REG_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(GYRO_CONFIG, 0x00);
  writeByte(ACCEL_CONFIG, 0x00);   // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_REG_USER_CTRL, 0x40);    // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
  writeByte(MPU9250_REG_FIFO_EN, 0x78);
  Thread::sleep(40);

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_REG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPUREG_FIFO_COUNTH, data, 2);    // read FIFO sample count
  fifo_count = ( (uint16_t) data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++) {
    int16_t acc_temp[3] = {0, 0, 0};
    int16_t gyro_temp[3]  = {0, 0, 0};

    readBytes(MPUREG_FIFO_R_W, data, 12);  // read data for averaging
    // Form signed 16-bit integer for each sample in FIFO
    acc_temp[0] = (int16_t) (( (int16_t) data[0] << 8)  | data[1]);
    acc_temp[1] = (int16_t) (( (int16_t) data[2] << 8)  | data[3]);
    acc_temp[2] = (int16_t) (( (int16_t) data[4] << 8)  | data[5]);
    gyro_temp[0]  = (int16_t) (( (int16_t) data[6] << 8)  | data[7]);
    gyro_temp[1]  = (int16_t) (( (int16_t) data[8] << 8)  | data[9]);
    gyro_temp[2]  = (int16_t) (( (int16_t) data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    acc_bias_[0] += (int32_t) acc_temp[0];
    acc_bias_[1] += (int32_t) acc_temp[1];
    acc_bias_[2] += (int32_t) acc_temp[2];
    gyro_bias_[0]  += (int32_t) gyro_temp[0];
    gyro_bias_[1]  += (int32_t) gyro_temp[1];
    gyro_bias_[2]  += (int32_t) gyro_temp[2];
  }

  // Normalize sums to get average count biases
  acc_bias_[0] /= (int32_t) packet_count;
  acc_bias_[1] /= (int32_t) packet_count;
  acc_bias_[2] /= (int32_t) packet_count;
  gyro_bias_[0]  /= (int32_t) packet_count;
  gyro_bias_[1]  /= (int32_t) packet_count;
  gyro_bias_[2]  /= (int32_t) packet_count;


  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  // Biases are additive, so change sign on calculated average gyro biases
  data[0] = (-gyro_bias_[0]/4  >> 8) & 0xFF;
  data[1] = (-gyro_bias_[0]/4)       & 0xFF;
  data[2] = (-gyro_bias_[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias_[1]/4)       & 0xFF;
  data[4] = (-gyro_bias_[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias_[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPUREG_XG_OFFS_USRH, data[0]);
  writeByte(MPUREG_XG_OFFS_USRL, data[1]);
  writeByte(MPUREG_YG_OFFS_USRH, data[2]);
  writeByte(MPUREG_YG_OFFS_USRL, data[3]);
  writeByte(MPUREG_ZG_OFFS_USRH, data[4]);
  writeByte(MPUREG_ZG_OFFS_USRL, data[5]);

  // Set scaled gyro biases
  gyro_bias_[0] = static_cast<double>(gyro_bias_[0])/static_cast<double>(gyrosensitivity);
  gyro_bias_[1] = static_cast<double>(gyro_bias_[1])/static_cast<double>(gyrosensitivity);
  gyro_bias_[2] = static_cast<double>(gyro_bias_[2])/static_cast<double>(gyrosensitivity);

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers.
  // These registers contain factory trim values which must be added to the
  // calculated accelerometer biases; on boot up these registers will hold non-zero values.
  // In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g,
  // so that the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0};   // A place to hold the factory accelerometer trim biases
  readBytes(MPUREG_XA_OFFSET_H, data, 2);   // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPUREG_YA_OFFSET_H, data, 2);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPUREG_ZA_OFFSET_H, data, 2);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++) {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[0] -= (acc_bias_[0]/8);
  accel_bias_reg[1] -= (acc_bias_[1]/8);
  accel_bias_reg[2] -= (acc_bias_[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[5] = data[5] | mask_bit[2];

  // Push accelerometer biases to hardware registers
  writeByte(MPUREG_XA_OFFSET_H, data[0]);
  writeByte(MPUREG_XA_OFFSET_L, data[1]);
  writeByte(MPUREG_YA_OFFSET_H, data[2]);
  writeByte(MPUREG_YA_OFFSET_L, data[3]);
  writeByte(MPUREG_ZA_OFFSET_H, data[4]);
  writeByte(MPUREG_ZA_OFFSET_L, data[5]);

  // Set scaled accelerometer biases
  acc_bias_[0] = static_cast<double>(acc_bias_[0])/static_cast<double>(accelsensitivity);
  acc_bias_[1] = static_cast<double>(acc_bias_[1])/static_cast<double>(accelsensitivity);
  acc_bias_[2] = static_cast<double>(acc_bias_[2])/static_cast<double>(accelsensitivity);
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

  readBytes(kAccelXoutH, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    data = static_cast<double>(bit_data);
    accel_data_[i] = data/acc_divider_ - acc_bias_[i];
  }
}

void MPU9250::getGyroData()
{
  uint8_t response[6];
  int16_t bit_data;
  double data;
  int i;

  readBytes(kGyroXoutH, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    data = static_cast<double>(bit_data);
    gyro_data_[i] = data/gyro_divider_ - gyro_bias_[i];
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
