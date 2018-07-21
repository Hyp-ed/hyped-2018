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
#ifdef PROXI
#include "sensors/vl6180.hpp"

#include <cstdint>

#include "utils/logger.hpp"
#include "utils/timer.hpp"
#include "utils/concurrent/thread.hpp"


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
constexpr uint16_t kModeStartStop                      = 0x01;
constexpr uint16_t kModeContinuous                     = 0x02;
constexpr uint16_t kResultInterruptStatusGpio          = 0x4F;
constexpr uint16_t kSystemInterruptConfigGpio          = 0x014;

namespace hyped {

using utils::io::I2C;
using utils::concurrent::Thread;

namespace sensors {

VL6180::VL6180(uint8_t i2c_addr, Logger& log)
    : log_(log),
      i2c_addr_(i2c_addr),
      i2c_(I2C::getInstance()),
      is_online_(false),
      timeout_(false)
{
  // Create I2C instance get register address
  turnOn();
  log_.INFO("VL6180", "Creating a sensor with id: %d", i2c_addr);
}

VL6180::~VL6180()
{
  // turn off ranging
  writeByte(kSysrangeStart, 0x01);
  Thread::sleep(100);
  log_.INFO("VL6180", "Stop Ranging command");
}

void VL6180::turnOn()
{
  log_.INFO("VL6180", "Trying to turn sensor on");

  if (!waitDeviceBooted()) {
    is_online_ = false;
    return;
  }

  writeByte(kSysrangeStart, 0x01);
  Thread::sleep(100);
  log_.INFO("VL6180", "Stop ranging command");

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

  writeByte(kSystemInterruptConfigGpio, 0x24);

  // Set max convergence time (Recommended default 50ms)
  uint8_t time_ms = 50;  // changes here
  setMaxConvergenceTime(time_ms);

  // Clear interrupt
  writeByte(kSystemInterruptClear, 0x01);
  log_.INFO("VL6180", "Clear interrupt");

  writeByte(kSysrangeStart, 0x01);

  if (isOnline()) {
    // TODO(Jack) Redo this, there has to be a better way
    log_.INFO("VL6180", "Sensor is online");
  } else {
    log_.ERR("VL6180", "Sensor is not operational");
  }
}

void VL6180::setMaxConvergenceTime(uint8_t time_ms)
{
  writeByte(kSysrangeMaxConvergenceTime, time_ms);
}

void VL6180::getData(Proximity* proxi)
{
  proxi->val = continuousRangeDistance();
  proxi->operational = is_online_;
}

void VL6180::startRanging()
{
  if (!writeByte(kSysrangeStart, 0x01)) {
    is_online_ = false;
  } else {
    is_online_ = true;
  }
}


bool VL6180::isOnline()
{
  uint8_t data;
  uint8_t status;

  if (!readByte(kResultRangeStatus, &data)) {
    is_online_ = false;
    log_.ERR("Vl6180", "No I2C connection");
    return false;
  }
  status = data >> 4;

  if (status == 0) {
    is_online_ = true;
  } else if (status != 0) {
    checkStatus();
    is_online_ = false;
  }
  return is_online_;
}

uint8_t VL6180::continuousRangeDistance()
{
  if (!is_online_) return 255;
  uint64_t start = utils::Timer::getTimeMicros();
  uint8_t data = 1;
  uint8_t interrupt = 1;
  uint64_t timeout = 30000;   // micro s
  // Make sure we are in continuous ranging mode
  readByte(kResultInterruptStatusGpio, &interrupt);

  while ((interrupt & 0x04) == 0) {
    if (!readByte(kResultInterruptStatusGpio, &interrupt)) {
      log_.ERR("Vl6180", "No I2C connection");
      is_online_ = false;
      return 0;
    }
    if ((utils::Timer::getTimeMicros() - start) > timeout) {
      log_.ERR("Vl6180", "TIMEOUT");
      is_online_ = false;
      timeout_ = true;
      return 255;
    }
  }
  readByte(kResultRangeVal, &data);   // read the sampled data
  log_.DBG3("VL6180", "Sensor continuous range: %f\n", data);
  writeByte(kSystemInterruptClear, 0x01);
  isOnline();
  return data;
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
      log_.ERR("VL6180", "Range < 0");
    break;
    case 15:
      log_.ERR("VL6180", "Result is out of range. This occurs typically around 200 mm");
    break;
    default:
          log_.ERR("VL6180", "Unidentified error");
  }
}

bool VL6180::waitDeviceBooted()
{
  // Will hold the return value of the register kSystemFreshOutOfReset
  uint8_t fresh_out_of_reset;
  int send_counter;

  for (send_counter = 0; send_counter < 5; send_counter++) {
    if (!readByte(kSystemFreshOutOfReset, &fresh_out_of_reset)) {
      log_.ERR("VL6180", "No I2C connection");
      is_online_ = false;
      return false;
    }
    if (fresh_out_of_reset == 1) {
      log_.INFO("VL6180", "Sensor out of reset");
      return true;
    }
    log_.DBG("VL6180", "Sensor failed to get out of reset");
    Thread::sleep(20);
  }
  log_.ERR("VL6180", "Sensor failed to get of reset");
  is_online_ = false;
  return false;
}

bool VL6180::readByte(uint16_t reg_add, uint8_t *data)
{
  uint8_t buffer[2];
  buffer[0] = reg_add >> 8;
  buffer[1] = reg_add & 0xFF;

  i2c_.write(i2c_addr_, buffer, 2);
  return i2c_.read(i2c_addr_, data, 1);
}

bool VL6180::writeByte(uint16_t reg_add, char data)
{
  uint8_t buffer[3];
  buffer[0]=reg_add>>8;
  buffer[1]=reg_add&0xFF;
  buffer[2]=data;

  return i2c_.write(i2c_addr_, buffer, 3);
}

}}   // namespace hyped::sensors

#endif
