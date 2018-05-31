/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 18/04/18
 * Description: Main file for VL6180
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
#include "sensors/vl6180.hpp"

#include <chrono>
#include <cstdint>

#include "utils/logger.hpp"



// Register addresses
constexpr uint16_t kSystemModeGpio0                    = 0x0010;
constexpr uint16_t kSystemModeGpio1                    = 0x0011;
constexpr uint16_t kSystemHistoryCtrl                  = 0x0012;
constexpr uint16_t kSystemInterruptConfigGpio          = 0x0014;
constexpr uint16_t kSystemInterruptClear               = 0x0015;
constexpr uint16_t kSystemFreshOutOfReset              = 0x0016;
constexpr uint16_t kSystemGroupedParameterHold         = 0x0017;
constexpr uint16_t kSysrangeStart                      = 0x0018;
constexpr uint16_t kSysrangeThreshHigh                 = 0x0019;
constexpr uint16_t kSysrangeThreshLow                  = 0x001A;
constexpr uint16_t kSysrangeIntermeasurementPeriod     = 0x001B;
constexpr uint16_t kSysrangeMaxConvergenceTime         = 0x001C;
constexpr uint16_t kSysrangeVhvRecalibrate             = 0x002E;
constexpr uint16_t kSysrangeVhvRepeatRate              = 0x0031;
constexpr uint16_t kResultRangeStatus                  = 0x004D;
constexpr uint16_t kResultRangeVal                     = 0x0062;
constexpr uint16_t kRangeDeviceReadyMask               = 0x01;
constexpr uint16_t kModeStartStop                      = 0x01;
constexpr uint16_t kModeContinuous                     = 0x02;
constexpr uint16_t kModeSingleShot                     = 0x00;
constexpr uint16_t kResultInterruptStatusGpio          = 0x4F;
constexpr uint16_t kInterruptClearRanging              = 0x01;
constexpr uint16_t kResIntRangeMask                    = 0x07;

namespace hyped {
namespace sensors {

VL6180::VL6180(uint8_t i2c_addr, Logger& log)
    : log_(log),
    on_(false),
    continuous_mode_(false)
{
  // Create I2C instance get register address
  this->i2c_addr_ = i2c_addr;
  this->turnOn();
  log_.INFO("VL6180", "Creating a sensor with id: %d", i2c_addr);
}

VL6180::~VL6180()
{
  this->turnOff();
  log_.INFO("VL6180", "Deconstructing sensor object");
}

void VL6180::turnOn()
{
  // return if already on
  if (this->on_) {
    log_.DBG("VL6180", "Sensor is already on\n");
    return;
  }

  // This waits for the device to be fresh out of reset (same thing as above)
  this->waitDeviceBooted();

  // Initialise the sensor / register tuning
  // Taken from ST Microelectronics API
  this->writeByte(0x0207, 0x01);
  this->writeByte(0x0208, 0x01);
  this->writeByte(0x0096, 0x00);
  this->writeByte(0x0097, 0xfd);
  this->writeByte(0x00e3, 0x00);
  this->writeByte(0x00e4, 0x04);
  this->writeByte(0x00e5, 0x02);
  this->writeByte(0x00e6, 0x01);
  this->writeByte(0x00e7, 0x03);
  this->writeByte(0x00f5, 0x02);
  this->writeByte(0x00d9, 0x05);
  this->writeByte(0x00db, 0xce);
  this->writeByte(0x00dc, 0x03);
  this->writeByte(0x00dd, 0xf8);
  this->writeByte(0x009f, 0x00);
  this->writeByte(0x00a3, 0x3c);
  this->writeByte(0x00b7, 0x00);
  this->writeByte(0x00bb, 0x3c);
  this->writeByte(0x00b2, 0x09);
  this->writeByte(0x00ca, 0x09);
  this->writeByte(0x0198, 0x01);
  this->writeByte(0x01b0, 0x17);
  this->writeByte(0x01ad, 0x00);
  this->writeByte(0x00ff, 0x05);
  this->writeByte(0x0100, 0x05);
  this->writeByte(0x0199, 0x05);
  this->writeByte(0x01a6, 0x1b);
  this->writeByte(0x01ac, 0x3e);
  this->writeByte(0x01a7, 0x1f);
  this->writeByte(0x0030, 0x00);

  // Perform a single temperature calibration of the ranging sensor
  writeByte(0x002e, 0x01);

  // Enables polling for New Sample ready when measurement completes
  this->writeByte(0x0011, 0x10);

  // Set the averaging sample period (datasheet recommends 48)
  this->writeByte(0x010a, 0x30);

  // Sets the Number of range measurements after which auto calibration of
  // system is performed (currently 255)
  this->writeByte(0x0031, 0xFF);

  // Perform a single recalibration
  this->writeByte(kSysrangeVhvRecalibrate, 0x01);

  // Set max convergence time (Recommended default 50ms)
  uint8_t time_ms = 50;  // changes here
  this->setMaxConvergenceTime(time_ms);

  this->on_ = true;
  log_.DBG("VL6180", "Sensor is on\n");
}

void VL6180::setMaxConvergenceTime(uint8_t time_ms)
{
  this->writeByte(kSysrangeMaxConvergenceTime, time_ms);
}

void VL6180::turnOff()
{
  this->on_ = false;
  log_.DBG("VL6180", "Sensor is now off\n");
}

double VL6180::getDistance()
{
  if (continuous_mode_) {
    return continuousRangeDistance();
  } else {
    return singleRangeDistance();
  }
}


void VL6180::setContinuousRangingMode()
{
  if (this->continuous_mode_ == true) {
    log_.DBG("VL6180", "Sensor already in continuous ranging mode\n");
    return;
  }
  // Write to sensor and set to continuous ranging mode
  this->writeByte(kSysrangeStart, kModeStartStop | kModeContinuous);

  uint8_t inter_measurement_time = 1;
  writeByte(kSysrangeIntermeasurementPeriod, inter_measurement_time);
  this->continuous_mode_ = true;
}

double VL6180::continuousRangeDistance()
{
  uint8_t data;
  data = 1;
  readByte(kResultRangeVal, &data);   // read the sampled data
  return static_cast<int>(data);
}

void VL6180::setSingleShotMode()
{
  if (this->continuous_mode_ == false) {
    log_.DBG("VL6180", "Sensor already in single shot mode\n");
    return;
  } else {
    // Write to sensor and set to single shot ranging mode
    writeByte(kSysrangeStart, kModeStartStop | kModeSingleShot);
    this->continuous_mode_ = false;
  }
}

double VL6180::singleRangeDistance()
{
  uint8_t data;
  data = 1;
  uint8_t status;
  status = 1;

  // Make sure in single shot ranging mode
  writeByte(kSysrangeStart, kModeStartStop | kModeSingleShot);

  // Clear the interrupt
  writeByte(kSystemInterruptClear, kInterruptClearRanging);

  // Wait until the sample is ready
  do {
    // Loops until device is ready
    rangeWaitDeviceReady();
    readByte(kResultInterruptStatusGpio, &status);
  }while(status);

  // Clear interrupt again
  writeByte(kSystemInterruptClear, kInterruptClearRanging);

  readByte(kResultRangeVal, &data);
  return static_cast<int>(data);
}

bool VL6180::waitDeviceBooted()
{
  // Will hold the return value of the register kSystemFreshOutOfReset
  uint8_t fresh_out_of_reset;
  do
  {
    this->readByte(kSystemFreshOutOfReset, &fresh_out_of_reset);
  }
  while (fresh_out_of_reset != 1);
  return true;
}

bool VL6180::rangeWaitDeviceReady()
{
  uint8_t data;
  while (true) {
    readByte(kResultRangeStatus, &data);
    data= data & kRangeDeviceReadyMask;
    if (data)
      return true;
  }
  return false;
}

int VL6180::readByte(uint16_t reg_add, uint8_t *data)
{
  uint8_t buffer[2];
  buffer[0] = reg_add >> 8;
  buffer[1] = reg_add & 0xFF;

  i2c_.write(this->i2c_addr_, buffer, 2);
  i2c_.read(i2c_addr_, data, 1);

  return 1;
}

int VL6180::writeByte(uint16_t reg_add, char data)
{
  uint8_t buffer[3];
  buffer[0]=reg_add>>8;
  buffer[1]=reg_add&0xFF;
  buffer[2]=data;

  i2c_.write(this->i2c_addr_, buffer, 3);
  return 1;
}

}}   // namespace hyped::sensors
