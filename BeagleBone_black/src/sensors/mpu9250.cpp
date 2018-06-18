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
constexpr uint8_t kAccelXoutH               = 0x3B;

constexpr uint8_t kMpuXaOffsetH             = 0x77;
constexpr uint8_t kMpuXaOffsetL             = 0x78;
constexpr uint8_t kMpuYaOffsetH             = 0x7A;
constexpr uint8_t kMpuYaOffsetL             = 0x7B;
constexpr uint8_t kMpuZaOffsetH             = 0x7D;
constexpr uint8_t kMpuZaOffsetL             = 0x7E;

constexpr uint8_t kAccelConfig              = 0x1C;
constexpr uint8_t kAccelConfig2             = 0x1D;

// gyroscope addresses
constexpr uint8_t  kGyroXoutH               = 0x43;

constexpr uint8_t kMpuXgOffsetUsrh          = 0x13;
constexpr uint8_t kMpuXgOffsetUsrl          = 0x14;
constexpr uint8_t kMpuYgOffsetUsrh          = 0x15;
constexpr uint8_t kMpuYgOffsetUsrl          = 0x16;
constexpr uint8_t kMpuZgOffsetUsrh          = 0x17;
constexpr uint8_t kMpuZgOffsetUsrl          = 0x18;

constexpr uint8_t  kGyroConfig              = 0x1B;

constexpr uint8_t kWhoAmIMpu9250            = 0x75;
constexpr uint8_t kWhoAmIResetValue1        = 0x71;
constexpr uint8_t kWhoAmIResetValue2        = 0x73;

// User Control
constexpr uint8_t kMpuRegUserCtrl           = 0x6A;

// I2C Master Control
constexpr uint8_t kMpuRegI2cMstCtrl         = 0x24;

// Power Management
constexpr uint8_t kMpuRegPwrMgmt1           = 0x6B;
constexpr uint8_t kMpuRegPwrMgmt2           = 0x6C;

// Configuration
constexpr uint8_t kMpuRegConfig             = 0x1A;

// Sample Rate Divider
constexpr uint8_t kMpuRegSmplrtDiv          = 0x19;

constexpr uint8_t kMpuRegFifoCountH         = 0x72;

// FIFO Enable
constexpr uint8_t kMpuRegFifoEn             = 0x23;

// Interrupt Enable
constexpr uint8_t kMpuRegIntEnable          = 0x38;

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

// Configuration bits mpu9250
constexpr uint8_t kMpuRegFifoRW            = 0x74;

// Resets the device to defaults
constexpr uint8_t kBitHReset                = 0x80;

namespace hyped {

utils::io::gpio::Direction kDirection = utils::io::gpio::kOut;
using utils::concurrent::Thread;

namespace sensors {

const uint64_t MPU9250::time_start = utils::Timer::getTimeMicros();

MPU9250::MPU9250(Logger& log, uint32_t pin, uint8_t acc_scale, uint8_t gyro_scale)
    : log_(log),
    gpio_(pin, kDirection, log),
    acc_scale_(acc_scale),
    gyro_scale_(gyro_scale)
{
  init();
  log_.INFO("MPU9250", "Creating a sensor with id: %d", 1);
}

void MPU9250::init()
{
  // Set pin high
  gpio_.set();

  // calibrateSensors();

  // Test connection
  while (!whoAmI());

  writeByte(kMpuRegPwrMgmt1, kBitHReset);   // Reset Device
  Thread::sleep(200);
  writeByte(kMpuRegUserCtrl, 0x20);   // set I2C_IF_DIS to disable slave mode I2C bus
  writeByte(kMpuRegPwrMgmt1, 0x01);          // Clock Source
  writeByte(kMpuRegPwrMgmt2, 0x00);          // Enable Acc & Gyro
  writeByte(kMpuRegConfig, 0x01);
  writeByte(kGyroConfig, 0x00);
  writeByte(kAccelConfig, 0x00);
  writeByte(kAccelConfig2, 0x01);
  setAcclScale(acc_scale_);
  setGyroScale(gyro_scale_);
}

void MPU9250::calibrateSensors()
{
  uint8_t data[12];
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(kMpuRegPwrMgmt1, kBitHReset);  // Write a one to bit 7 reset bit; toggle reset device
  Thread::sleep(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(kMpuRegPwrMgmt1, 0x01);
  writeByte(kMpuRegPwrMgmt2, 0x00);
  Thread::sleep(200);

  // Configure device for bias calculation
  writeByte(kMpuRegIntEnable, 0x00);    // Disable all interrupts
  writeByte(kMpuRegFifoEn, 0x00);       // Disable FIFO
  writeByte(kMpuRegPwrMgmt1, 0x00);    // Turn on internal clock source
  writeByte(kMpuRegI2cMstCtrl, 0x00);  // Disable I2C master
  writeByte(kMpuRegUserCtrl, 0x00);     // Disable FIFO and I2C master modes
  writeByte(kMpuRegUserCtrl, 0x0C);     // Reset FIFO and DMP
  Thread::sleep(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(kMpuRegConfig, 0x01);       // Set low-pass filter to 188 Hz
  writeByte(kMpuRegSmplrtDiv, 0x00);   // Set sample rate to 1 kHz
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(kGyroConfig, 0x00);
  writeByte(kAccelConfig, 0x00);   // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(kMpuRegUserCtrl, 0x40);    // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
  writeByte(kMpuRegFifoEn, 0x78);
  Thread::sleep(40);

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(kMpuRegFifoEn, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(kMpuRegFifoCountH, data, 2);    // read FIFO sample count
  fifo_count = ( (uint16_t) data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++) {
    int16_t acc_temp[3] = {0, 0, 0}, gyro_temp[3]  = {0, 0, 0};

    readBytes(kMpuRegFifoRW, data, 12);  // read data for averaging
    // Form signed 16-bit integer for each sample in FIFO
    acc_temp[0] = (int16_t) (( (int16_t) data[0] << 8)  | data[1]);
    acc_temp[1] = (int16_t) (( (int16_t) data[2] << 8)  | data[3]);
    acc_temp[2] = (int16_t) (( (int16_t) data[4] << 8)  | data[5]);
    gyro_temp[0]  = (int16_t) (( (int16_t) data[6] << 8)  | data[7]);
    gyro_temp[1]  = (int16_t) (( (int16_t) data[8] << 8)  | data[9]);
    gyro_temp[2]  = (int16_t) (( (int16_t) data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] += (int32_t) acc_temp[0];
    accel_bias[1] += (int32_t) acc_temp[1];
    accel_bias[2] += (int32_t) acc_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;


  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  // Biases are additive, so change sign on calculated average gyro biases
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(kMpuXgOffsetUsrh, data[0]);
  writeByte(kMpuXgOffsetUsrl, data[1]);
  writeByte(kMpuYgOffsetUsrh, data[2]);
  writeByte(kMpuYgOffsetUsrl, data[3]);
  writeByte(kMpuZgOffsetUsrh, data[4]);
  writeByte(kMpuZgOffsetUsrl, data[5]);

  // Set scaled gyro biases
  gyro_bias_[0] = static_cast<float>(gyro_bias[0])/static_cast<float>(gyrosensitivity);
  gyro_bias_[1] = static_cast<float>(gyro_bias[1])/static_cast<float>(gyrosensitivity);
  gyro_bias_[2] = static_cast<float>(gyro_bias[2])/static_cast<float>(gyrosensitivity);

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers.
  // These registers contain factory trim values which must be added to the
  // calculated accelerometer biases; on boot up these registers will hold non-zero values.
  // In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g,
  // so that the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0};   // A place to hold the factory accelerometer trim biases
  readBytes(kMpuXaOffsetH, data, 2);   // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(kMpuYaOffsetH, data, 2);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(kMpuZaOffsetH, data, 2);
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
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

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
  writeByte(kMpuXaOffsetH, data[0]);
  writeByte(kMpuXaOffsetL, data[1]);
  writeByte(kMpuYaOffsetH, data[2]);
  writeByte(kMpuYaOffsetL, data[3]);
  writeByte(kMpuZaOffsetH, data[4]);
  writeByte(kMpuZaOffsetL, data[5]);

  // Set scaled accelerometer biases
  acc_bias_[0] = static_cast<float>(accel_bias[0])/static_cast<float>(accelsensitivity);
  acc_bias_[1] = static_cast<float>(accel_bias[1])/static_cast<float>(accelsensitivity);
  acc_bias_[2] = static_cast<float>(accel_bias[2])/static_cast<float>(accelsensitivity);
}

bool MPU9250::whoAmI()
{
  uint8_t data;

  // Who am I checks what address the sensor is at
  readByte(kWhoAmIMpu9250, &data);
  log_.ERR("MPU9250", "who am I: %u", data);
  if (data != kWhoAmIResetValue1 && data != kWhoAmIResetValue2) {
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
    select();
    spi_.write(write_reg, &write_data, 1);
    deSelect();
}

void MPU9250::readByte(uint8_t read_reg, uint8_t *read_data)
{
    select();
    spi_.read(read_reg | kReadFlag, read_data, 1);
    deSelect();
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
  float data;
  int i;

  readBytes(kAccelXoutH, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    data = static_cast<float>(bit_data);
    accel_data_[i] = data/acc_divider_  * 9.80665;   // - acc_bias_[i];
  }
}

void MPU9250::getGyroData()
{
  uint8_t response[6];
  int16_t bit_data;
  float data;
  int i;

  readBytes(kGyroXoutH, response, 6);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
    data = static_cast<float>(bit_data);
    gyro_data_[i] = data/gyro_divider_;    // - gyro_bias_[i];
  }
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
  getGyroData();
  getAcclData();
  auto& acc = imu->acc.value;
  auto& gyr = imu->gyr.value;
  acc[0] = accel_data_[0];
  acc[1] = accel_data_[1];
  acc[2] = accel_data_[2];
  gyr[0] = gyro_data_[0];
  gyr[1] = gyro_data_[1];
  gyr[2] = gyro_data_[2];

  uint32_t time = utils::Timer::getTimeMillis() - (time_start/1000);
  imu->acc.timestamp = time;
  imu->gyr.timestamp = time;
}

}}   // namespace hyped::sensors
