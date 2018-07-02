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

#include <cstdint>

#include "utils/logger.hpp"
#include "utils/timer.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/math/statistics.hpp"


// Register addresses
constexpr uint16_t kIdentificationModelId              = 0x000;
constexpr uint16_t kSystemInterruptClear               = 0x0015;
constexpr uint16_t kSystemFreshOutOfReset              = 0x0016;
constexpr uint16_t kSysrangeStart                      = 0x0018;
constexpr uint16_t kSysrangeIntermeasurementPeriod     = 0x001B;
constexpr uint16_t kSysrangeMaxConvergenceTime         = 0x001C;
constexpr uint16_t kSysrangeVhvRecalibrate             = 0x002E;
constexpr uint16_t kResultRangeStatus                  = 0x004D;
constexpr uint16_t kResultRangeVal                     = 0x0062;
constexpr uint16_t kRangeDeviceReadyMask               = 0x01;
constexpr uint16_t kModeStartStop                      = 0x01;
constexpr uint16_t kModeContinuous                     = 0x02;
constexpr uint16_t kModeSingleShot                     = 0x00;
constexpr uint16_t kResultInterruptStatusGpio          = 0x4F;
constexpr uint16_t kInterruptClearRanging              = 0x01;

namespace hyped {

using utils::io::I2C;
using utils::concurrent::Thread;
using utils::math::OnlineStatistics;

namespace sensors {

VL6180::VL6180(uint8_t i2c_addr, Logger& log)
    : log_(log),
      continuous_mode_(false),
      i2c_addr_(i2c_addr),
      i2c_(I2C::getInstance()),
      is_online_(false)
{
  // Create I2C instance get register address
  turnOn();
  log_.INFO("VL6180", "Creating a sensor with id: %d", i2c_addr);
}

void VL6180::setAddress(uint32_t i2c_addr)
{
  writeByte(0x0212, i2c_addr);
  i2c_addr_ = i2c_addr;
}

void VL6180::turnOn()
{
  // This waits for the device to be fresh out of reset
  waitDeviceBooted();

  // Initialise the sensor / register tuning
  // Taken from ST Microelectronics API
  writeByte(0x0207, 0x01);
  writeByte(0x0208, 0x01);
  writeByte(0x0096, 0x00);
  writeByte(0x0097, 0xfd);
  writeByte(0x00e3, 0x00);
  writeByte(0x00e4, 0x04);
  writeByte(0x00e5, 0x02);
  writeByte(0x00e6, 0x01);
  writeByte(0x00e7, 0x03);
  writeByte(0x00f5, 0x02);
  writeByte(0x00d9, 0x05);
  writeByte(0x00db, 0xce);
  writeByte(0x00dc, 0x03);
  writeByte(0x00dd, 0xf8);
  writeByte(0x009f, 0x00);
  writeByte(0x00a3, 0x3c);
  writeByte(0x00b7, 0x00);
  writeByte(0x00bb, 0x3c);
  writeByte(0x00b2, 0x09);
  writeByte(0x00ca, 0x09);
  writeByte(0x0198, 0x01);
  writeByte(0x01b0, 0x17);
  writeByte(0x01ad, 0x00);
  writeByte(0x00ff, 0x05);
  writeByte(0x0100, 0x05);
  writeByte(0x0199, 0x05);
  writeByte(0x01a6, 0x1b);
  writeByte(0x01ac, 0x3e);
  writeByte(0x01a7, 0x1f);
  writeByte(0x0030, 0x00);

  // Perform a single temperature calibration of the ranging sensor
  writeByte(0x002e, 0x01);

  // Enables polling for New Sample ready when measurement completes
  writeByte(0x0011, 0x10);

  // Set the averaging sample period (datasheet recommends 48)
  writeByte(0x010a, 0x30);

  // Sets the Number of range measurements after which auto calibration of
  // system is performed (currently 255)
  writeByte(0x0031, 0xFF);

  // Perform a single recalibration
  writeByte(kSysrangeVhvRecalibrate, 0x01);

  // Set max convergence time (Recommended default 50ms)
  uint8_t time_ms = 50;  // changes here
  setMaxConvergenceTime(time_ms);

  if (isOnline()) {
    log_.INFO("VL6180", "Sensor is online");
  } else {
    log_.ERR("VL6180", "Sensor is not operational");
  }
}

float VL6180::calcCalibrationData()
{
  if (is_online_) {
    OnlineStatistics<float> stats = OnlineStatistics<float>();
    for (int i = 0; i < 100; i++) {
      stats.update(getDistance());
      Thread::sleep(9);
    }
    log_.INFO("VL6180", "Sensor has calculated the variance");
    return stats.getVariance();
  } else {
    log_.ERR("VL6180", "Could not calibrate proxi, sensor not operational");
    return -1.0;
  }
}

void VL6180::setMaxConvergenceTime(uint8_t time_ms)
{
  writeByte(kSysrangeMaxConvergenceTime, time_ms);
}

uint8_t VL6180::getDistance()
{
  // If sensor is not online try and turn on
  if (!is_online_) {
    turnOn();
  } else if (continuous_mode_) {
    return continuousRangeDistance();
  } else {
    return singleRangeDistance();
  }
}


bool VL6180::isOnline()
{
  uint8_t data;
  uint8_t status;

  readByte(kResultRangeStatus, &data);
  status = data >> 4;

  if (status == 0) {
    is_online_ = true;
  } else if (status != 0) {
    checkStatus();
    is_online_ = false;
  }

  // Check to see if i2c transaction is working by checking model ID
  // TODO(jack) check to see if this works
  readByte(kIdentificationModelId, &data);

  // Value should be 0xB4 after reset
  if (data != 0xB4) {
    log_.ERR("VL6180", "Data should of been: %d, but was %d", 0xB4, data);
    is_online_ = false;
  }
  log_.INFO("TEST", "data: %d", data);
  return is_online_;
}

void VL6180::setContinuousRangingMode()
{
  if (continuous_mode_) {
    log_.DBG("VL6180", "Sensor already in continuous ranging mode\n");
    return;
  }
  // Write to sensor and set to continuous ranging mode
  writeByte(kSysrangeStart, kModeStartStop | kModeContinuous);
  uint8_t inter_measurement_time = 1;
  writeByte(kSysrangeIntermeasurementPeriod, inter_measurement_time);
  continuous_mode_ = true;
  log_.INFO("VL6180", "Sensor is in continuous ranging mode\n");
}

uint8_t VL6180::continuousRangeDistance()
{
  uint64_t start = utils::Timer::getTimeMicros();
  uint8_t data = 1;
  uint8_t interrupt = 1;
  uint64_t timeout = 50;   // ms
  readByte(kResultInterruptStatusGpio, &interrupt);

  while ((interrupt & 0x04) == 0) {
    readByte(kResultInterruptStatusGpio, &interrupt);
    if ((start - utils::Timer::getTimeMicros()) > timeout) {
      return 255;
      is_online_ = false;
    }
  }

  readByte(kResultRangeVal, &data);   // read the sampled data
  log_.DBG3("VL6180", "Sensor continuous range: %f\n", data);
  writeByte(kSystemInterruptClear, 0x01);
  return data;
}

void VL6180::setSingleShotMode()
{
  if (!continuous_mode_) {
    log_.DBG("VL6180", "Sensor already in single shot mode\n");
    return;
  } else {
    // Write to sensor and set to single shot ranging mode
    writeByte(kSysrangeStart, kModeStartStop | kModeSingleShot);
    continuous_mode_ = false;
    log_.INFO("VL6180", "Sensor is in single-shot ranging mode\n");
  }
}

uint8_t VL6180::singleRangeDistance()
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
    rangeWaitDeviceReady();
    readByte(kResultInterruptStatusGpio, &status);
  }while(status);

  writeByte(kSystemInterruptClear, kInterruptClearRanging);
  readByte(kResultRangeVal, &data);
  log_.DBG3("VL6180", "Sensor single-shot range: %f\n", data);
  return data;
}

bool VL6180::waitDeviceBooted()
{
  // Will hold the return value of the register kSystemFreshOutOfReset
  uint8_t fresh_out_of_reset;
  int send_counter;

  for (send_counter = 0; send_counter < 3; send_counter++) {
    readByte(kSystemFreshOutOfReset, &fresh_out_of_reset);
    if (fresh_out_of_reset == 1) {
      log_.DBG("VL6180", "Sensor out of reset");
      return true;
    }
    Thread::sleep(100);
  }
  log_.ERR("VL6180", "Sensor failed to get of reset");
  is_online_ = false;
  return false;
}

bool VL6180::rangeWaitDeviceReady()
{
  uint8_t data;
  for (int i = 0; i < 10; i++) {
    readByte(kResultRangeStatus, &data);
    data = data & kRangeDeviceReadyMask;
    if (data)
      return true;
    Thread::yield();
  }
  log_.ERR("VL6180", "Sensor took too long to wait for data");
  is_online_ = false;
  return false;
}

void VL6180::checkStatus()
{
  uint8_t data;
  uint8_t status;
  // Check for an error in the error/status register
  readByte(kResultRangeStatus, &data);
  status = data >> 4;

    // Parse the error
  switch (status) {
    case 1:
      log_.ERR("VL6180", "System error detected. No measurement possible.");
    break;
    case 2:
      log_.ERR("VL6180", "System error detected. No measurement possible.");
    break;
    case 3:
      log_.ERR("VL6180", "System error detected. No measurement possible.");
    break;
    case 4:
      log_.ERR("VL6180", "System error detected. No measurement possible.");
    break;
    case 5:
      log_.ERR("VL6180", "System error detected. No measurement possible.");
    break;
    case 6:
      log_.ERR("VL6180", "Early convergence estimate check failed.");
    break;
    case 7:
      log_.ERR("VL6180", "System did not converge before the specified max.");
    break;
    case 8:
      log_.ERR("VL6180", "Ignore threshold check failed");
    break;
    case 11:
      log_.ERR("VL6180", "Ambient conditions too high. Measurement invalidated");
    break;
    case 12:
      log_.ERR("VL6180", "Range < 0");
    break;
    case 13:
      log_.ERR("VL6180", "Result is out of range. This occurs typically around 200 mm");
    break;
    case 14:
      log_.ERR("VL6180", "Range < 0 .");
    break;
    case 15:
      log_.ERR("VL6180", "Result is out of range. This occurs typically around 200 mm");
    break;
    default:
          log_.ERR("VL6180", "Unidentified error");
  }
}

void VL6180::readByte(uint16_t reg_add, uint8_t *data)
{
  uint8_t buffer[2];
  buffer[0] = reg_add >> 8;
  buffer[1] = reg_add & 0xFF;

  i2c_.write(i2c_addr_, buffer, 2);
  i2c_.read(i2c_addr_, data, 1);
}

void VL6180::writeByte(uint16_t reg_add, char data)
{
  uint8_t buffer[3];
  buffer[0]=reg_add>>8;
  buffer[1]=reg_add&0xFF;
  buffer[2]=data;

  i2c_.write(i2c_addr_, buffer, 3);
}

}}   // namespace hyped::sensors
