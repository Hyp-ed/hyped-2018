/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 18/04/18
 * Description: Main file for Vl6180
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
#include <thread>
#include <chrono>

// Register addresses
constexpr uint16_t IDENTIFICATION__MODEL_ID              = 0x0000;
constexpr uint16_t IDENTIFICATION__MODEL_REV_MAJOR       = 0x0001;
constexpr uint16_t IDENTIFICATION__MODEL_REV_MINOR       = 0x0002;
constexpr uint16_t IDENTIFICATION__MODULE_REV_MAJOR      = 0x0003;
constexpr uint16_t IDENTIFICATION__MODULE_REV_MINOR      = 0x0004;
constexpr uint16_t IDENTIFICATION__DATE_HI               = 0x0006;
constexpr uint16_t IDENTIFICATION__DATE_LO               = 0x0007;
constexpr uint16_t IDENTIFICATION__TIME                  = 0x0008;  // Could also use 0x0009
constexpr uint16_t SYSTEM__MODE_GPIO0                    = 0x0010;
constexpr uint16_t SYSTEM__MODE_GPIO1                    = 0x0011;
constexpr uint16_t SYSTEM__HISTORY_CTRL                  = 0x0012;
constexpr uint16_t SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x0014;
constexpr uint16_t SYSTEM__INTERRUPT_CLEAR               = 0x0015;
constexpr uint16_t SYSTEM__FRESH_OUT_OF_RESET            = 0x0016;
constexpr uint16_t SYSTEM__GROUPED_PARAMETER_HOLD        = 0x0017;
constexpr uint16_t SYSRANGE__START                       = 0x0018;
constexpr uint16_t SYSRANGE__THRESH_HIGH                 = 0x0019;
constexpr uint16_t SYSRANGE__THRESH_LOW                  = 0x001A;
constexpr uint16_t SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x001B;
constexpr uint16_t SYSRANGE__MAX_CONVERGENCE_TIME        = 0x001C;
constexpr uint16_t SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x001E;
constexpr uint16_t SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x0021;
constexpr uint16_t SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x0022;
constexpr uint16_t SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x0024;
constexpr uint16_t SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x0025;
constexpr uint16_t SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x0026;
constexpr uint16_t SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x002C;
constexpr uint16_t SYSRANGE__RANGE_CHECK_ENABLES         = 0x002D;
constexpr uint16_t SYSRANGE__VHV_RECALIBRATE             = 0x002E;
constexpr uint16_t SYSRANGE__VHV_REPEAT_RATE             = 0x0031;
constexpr uint16_t SYSALS__START                         = 0x0038;
constexpr uint16_t SYSALS__THRESH_HIGH                   = 0x003A;
constexpr uint16_t SYSALS__THRESH_LOW                    = 0x003C;
constexpr uint16_t SYSALS__INTERMEASUREMENT_PERIOD       = 0x003E;
constexpr uint16_t SYSALS__ANALOGUE_GAIN                 = 0x003F;
constexpr uint16_t SYSALS__INTEGRATION_PERIOD            = 0x0040;
constexpr uint16_t RESULT__RANGE_STATUS                  = 0x004D;
constexpr uint16_t RESULT__ALS_STATUS                    = 0x004E;
constexpr uint16_t RESULT__INTERRUPT_STATUS_GPIO         = 0x004F;
constexpr uint16_t RESULT__ALS_VAL                       = 0x0050;
constexpr uint16_t RESULT__HISTORY_BUFFER_x              = 0x0052;  // range 0x0052 to 0x0060
constexpr uint16_t RESULT__RANGE_VAL                     = 0x0062;
constexpr uint16_t RESULT__RANGE_RAW                     = 0x0064;
constexpr uint16_t RESULT__RANGE_RETURN_RATE             = 0x0066;
constexpr uint16_t RESULT__RANGE_REFERENCE_RATE          = 0x0068;
constexpr uint16_t RESULT__RANGE_RETURN_SIGNAL_COUNT     = 0x006C;
constexpr uint16_t RESULT__RANGE_REFERENCE_SIGNAL_COUNT  = 0x0070;
constexpr uint16_t RESULT__RANGE_RETURN_AMB_COUNT        = 0x0074;
constexpr uint16_t RESULT__RANGE_REFERENCE_AMB_COUNT     = 0x0078;
constexpr uint16_t RESULT__RANGE_RETURN_CONV_TIME        = 0x007C;
constexpr uint16_t RESULT__RANGE_REFERENCE_CONV_TIME     = 0x0080;
constexpr uint16_t READOUT__AVERAGING_SAMPLE_PERIOD      = 0x010A;
constexpr uint16_t FIRMWARE__BOOTUP                      = 0x0119;
constexpr uint16_t FIRMWARE__RESULT_SCALER               = 0x0120;
constexpr uint16_t I2C_SLAVE__DEVICE_ADDRESS             = 0x0212;
constexpr uint16_t INTERLEAVED_MODE__ENABLE              = 0x02A3;
constexpr uint16_t RANGE_DEVICE_READY_MASK               = 0x01;
constexpr uint16_t MODE_START_STOP                       = 0x01;
constexpr uint16_t MODE_CONTINUOUS                       = 0x02;

namespace hyped {
namespace sensors {

Vl6180::Vl6180(uint8_t id, Logger& log)
  :Thread(id, log)
{
  log_.INFO("VL6180", "Creating a sensor with id: %d", id);
}

Vl6180::~Vl6180()
{
  this->turnOff();
  log_.INFO("VL6180", "Deconstructing sensor object");
}

void Vl6180::turnOn()
{
  // return if already on
  if (this->on_) {
    log_.DBG("VL6180", "Sensor is already on\n");
    return;
  }

  // Wait incase the sensor has just been turned off
  // TODO(Anyone) need to check wait time

  // Turn on pins
  // TODO(Anyone) pin write to turn sensor on

  // Wait for 1.5ms (Data sheet says 1.4ms)
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

  // Might need to use these register access' trying to understand them
  // /* Recommended : Public registers - See data sheet for more detail */
  //   VL6180x_WrByte( dev, 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */
  //   /* Optional: Public registers - See data sheet for more detail */
  //   VL6180x_WrByte( dev, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
  //   VL6180x_WrByte( dev, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
  //   VL6180x_WrByte( dev, 0x0014, 0x24); /* Configures interrupt on New sample ready */
  //   status=VL6180x_RangeSetMaxConvergenceTime(dev, 50); /*  Calculate ece value on initialization (use max conv) */

  // Enables polling for New Sample ready when measurement completes
  this->writeByte(0x0011, 0x10);

  // Set the averaging sample period (datasheet recommends 48)
  this->writeByte(0x010a, 0x30);

  // Sets the Number of range measurements after which auto calibration of
  // system is performed (currently 127)
  this->writeByte(0x0031, 0xFF);

  // Perform a single recalibration
  this->writeByte(SYSRANGE__VHV_RECALIBRATE, 0x01);

  this->on_ = true;
  log_.DBG("VL6180", "Sensor is on\n");
}

void Vl6180::turnOff()
{
  // TODO(Anyone) do pin write to turn off vl6180
  this->on_ = false;
  log_.DBG("VL6180", "Sensor is now off\n");
}

double Vl6180::getDistance()
{
  uint8_t data;
  this->readByte(RESULT__RANGE_VAL, &data);
  return (int) data;
}

void Vl6180::setContinuousRangingMode()
{
  if ( this->continuous_mode_ == true ) {
    log_.DBG("VL6180", "Sensor already in continuous ranging mode\n");
    return;
  }

  // Write to sensor and set to continuous ranging mode
  this->writeByte(SYSRANGE__START, MODE_START_STOP | MODE_CONTINUOUS );
  this->continuous_mode_ = true;

}

bool Vl6180::waitDeviceBooted()
{
  // Will hold the return value of the register SYSTEM__FRESH_OUT_OF_RESET
  uint8_t fresh_out_of_reset;
  int status;

  do
  {
    status = this->readByte(SYSTEM__FRESH_OUT_OF_RESET, &fresh_out_of_reset);
  }
  while (fresh_out_of_reset != 1 && status == 0);

  return true;
}

bool Vl6180::rangeWaitDeviceReady()
{
  uint8_t data;

  while(true) {
    this->readByte(RESULT__RANGE_STATUS, &data);
    if(data == (data & RANGE_DEVICE_READY_MASK))
      return true;
  }
  return false;
}

int Vl6180::readByte(uint16_t reg_add, uint8_t *data)
{
  char __attribute__((unused)) buffer[2];
  buffer[0] = reg_add >> 8;
  buffer[1] = reg_add & 0xFF;
  char __attribute__((unused)) recv_buffer[1];

  // TODO(Anyone) write read I2C

  return 1;
}

int Vl6180::writeByte(uint16_t reg_add, char data)
{
  char __attribute__((unused)) buffer[3];
  buffer[0]=reg_add>>8;
  buffer[1]=reg_add&0xFF;
  buffer[2]=data;

  // TODO(Anyone) write I2C

  return 1;
}

}}   // namespace hyped::sensors
