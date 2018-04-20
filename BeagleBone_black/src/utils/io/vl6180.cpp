/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
 * Description:
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
#include "utils/io/vl6180.hpp"
#include <cstdint>
#include <thread>
#include <chrono>

// Register addresses
#define IDENTIFICATION__MODEL_ID              0x0000
#define IDENTIFICATION__MODEL_REV_MAJOR       0x0001
#define IDENTIFICATION__MODEL_REV_MINOR       0x0002
#define IDENTIFICATION__MODULE_REV_MAJOR      0x0003
#define IDENTIFICATION__MODULE_REV_MINOR      0x0004
#define IDENTIFICATION__DATE_HI               0x0006
#define IDENTIFICATION__DATE_LO               0x0007
#define IDENTIFICATION__TIME                  0x0008 // Could also use 0x0009
#define SYSTEM__MODE_GPIO0                    0x0010
#define SYSTEM__MODE_GPIO1                    0x0011
#define SYSTEM__HISTORY_CTRL                  0x0012
#define SYSTEM__INTERRUPT_CONFIG_GPIO         0x0014
#define SYSTEM__INTERRUPT_CLEAR               0x0015
#define SYSTEM__FRESH_OUT_OF_RESET            0x0016
#define SYSTEM__GROUPED_PARAMETER_HOLD        0x0017
#define SYSRANGE__START                       0x0018
#define SYSRANGE__THRESH_HIGH                 0x0019
#define SYSRANGE__THRESH_LOW                  0x001A
#define SYSRANGE__INTERMEASUREMENT_PERIOD     0x001B
#define SYSRANGE__MAX_CONVERGENCE_TIME        0x001C
#define SYSRANGE__CROSSTALK_COMPENSATION_RATE 0x001E
#define SYSRANGE__CROSSTALK_VALID_HEIGHT      0x0021
#define SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  0x0022
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET   0x0024
#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   0x0025
#define SYSRANGE__RANGE_IGNORE_THRESHOLD      0x0026
#define SYSRANGE__MAX_AMBIENT_LEVEL_MULT      0x002C
#define SYSRANGE__RANGE_CHECK_ENABLES         0x002D
#define SYSRANGE__VHV_RECALIBRATE             0x002E
#define SYSRANGE__VHV_REPEAT_RATE             0x0031
#define SYSALS__START                         0x0038
#define SYSALS__THRESH_HIGH                   0x003A
#define SYSALS__THRESH_LOW                    0x003C
#define SYSALS__INTERMEASUREMENT_PERIOD       0x003E
#define SYSALS__ANALOGUE_GAIN                 0x003F
#define SYSALS__INTEGRATION_PERIOD            0x0040
#define RESULT__RANGE_STATUS                  0x004D
#define RESULT__ALS_STATUS                    0x004E
#define RESULT__INTERRUPT_STATUS_GPIO         0x004F
#define RESULT__ALS_VAL                       0x0050
#define RESULT__HISTORY_BUFFER_x              0x0052 // range 0x0052 to 0x0060
#define RESULT__RANGE_VAL                     0x0062
#define RESULT__RANGE_RAW                     0x0064
#define RESULT__RANGE_RETURN_RATE             0x0066
#define RESULT__RANGE_REFERENCE_RATE          0x0068
#define RESULT__RANGE_RETURN_SIGNAL_COUNT     0x006C
#define RESULT__RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define RESULT__RANGE_RETURN_AMB_COUNT        0x0074
#define RESULT__RANGE_REFERENCE_AMB_COUNT     0x0078
#define RESULT__RANGE_RETURN_CONV_TIME        0x007C
#define RESULT__RANGE_REFERENCE_CONV_TIME     0x0080
#define READOUT__AVERAGING_SAMPLE_PERIOD      0x010A
#define FIRMWARE__BOOTUP                      0x0119
#define FIRMWARE__RESULT_SCALER               0x0120
#define I2C_SLAVE__DEVICE_ADDRESS             0x0212
#define INTERLEAVED_MODE__ENABLE              0x02A3




 namespace hyped {
 namespace utils {
 namespace io {

Vl6180::Vl6180(uint8_t id, Logger& log)
    :Thread(id, log)
{

  log_.INFO("VL6180", "Creating a sensor with id: " + id + "\n");
}



/**
  *  @brief  { turns on sensor }
  */

void Vl6180::turn_on()
{


  // return if already on
  if(this->on)
    log_.DBG("VL6180", "Sensor is already on\n");
    return;

  // Wait incase the sensor has just been turned off
  // TODO need to check wait time

  // Turn on pins TODO pin write to turn sensor on

  // Wait for 1.5ms (Data sheet says 1.4ms)
  //TODO check if there is a way to do this through HYP-ED threading
  std::this_thread::sleep_for(std::chrono::microseconds(ms*1400));

  this->on = true;
  log_.DBG("VL6180", "Sensor is on\n");

}

/**
  *  @brief  { turns off sensor }
  */

void Vl6180::turn_off()
{
  // TODO do pin write to turn off vl6180
  this->on = false;
  log_.DBG("VL6180", "Sensor is now off\n");
}

/**
  *  @brief  { checks the sensors state }
  */

bool Vl6180::is_on()
{
  return this->on;
}

/**
  *  @brief  { returns the distance from the nearest object
              the sensor is facing }
  */

int Vl6180::get_distance()
{

}



 }}}   // namespace hyped::utils::io
